#!/usr/bin/env python3
"""
Tether Constraint Manager con Controllo Adattivo Altezza - VERSIONE OTTIMIZZATA
================================================================================

MIGLIORAMENTI CHIAVE:
1. ✅ Isteresi CORRETTA e robusta (evita oscillazioni)
2. ✅ Stati intermedi per transizioni smooth (NORMAL → APPROACHING → LIFTING → CLEARING)
3. ✅ Controllo DIRETTO dell'altezza target dell'oggetto
4. ✅ Validazione parametri all'avvio
5. ✅ Logging migliorato per debug

Setup fisico:
- Pali: 95cm (0.95m) sopra i robot
- Corda: 1.5m totale
- Oggetto: punto medio della corda
"""

import math
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import OccupancyGrid


# ==================== UTILITY FUNCTIONS ====================

def yaw_from_quaternion(q) -> float:
    """Compute yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float]:
    """Return (z,w) quaternion components for pure yaw rotation."""
    half = 0.5 * yaw
    return math.sin(half), math.cos(half)


# ==================== DATA STRUCTURES ====================

class OperationMode(IntEnum):
    """Operation modes with smooth state transitions"""
    NORMAL = 0       # Formazione normale, nessun intervento
    APPROACHING = 1  # Ostacolo rilevato vicino, inizia preparazione
    LIFTING = 2      # Lifting attivo, oggetto completamente alzato
    CLEARING = 3     # Ostacolo superato, ritorno graduale a NORMAL


@dataclass
class RobotGoalState:
    """Stores the latest goal for a robot"""
    goal: Optional[PoseStamped] = None
    last_update_time: Optional[Time] = None


@dataclass
class ObstacleState:
    """Tracks obstacle detection with hysteresis (count- and/or time-based)."""
    detection_count: int = 0
    clear_count: int = 0
    last_detection_time: Optional[Time] = None
    closest_distance: float = float('inf')

    # Time-based confirm/clear timers
    detect_enter_time: Optional[Time] = None
    clear_enter_time: Optional[Time] = None

    # Early pre-check (confirm obstacle earlier without changing state yet)
    precheck_enter_time: Optional[Time] = None
    prechecked: bool = False

    # For re-trigger filtering (avoid returning to LIFTING due to noise/behind obstacles)
    prev_closest_distance: float = float('inf')
    clearing_mode_enter_time: Optional[Time] = None

# ==================== MAIN NODE ====================

class TetherConstraintManager(Node):
    """
    Gestisce vincolo del nastro con controllo adattivo dell'altezza.
    
    STATI OPERATIVI:
    - NORMAL: formazione normale, pass-through dei goal
    - APPROACHING: ostacolo rilevato, inizia ad allontanare i robot
    - LIFTING: lifting completo, oggetto a altezza target
    - CLEARING: ostacolo superato, transizione graduale a NORMAL
    """

    def __init__(self):
        super().__init__("tether_constraint_manager_optimized")

        # ==================== PARAMETERS ====================
        
        # Robot configuration
        self.declare_parameter("robot_namespaces", ["tb3_3", "tb3_4"])
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        
        # Physical setup parameters
        self.declare_parameter("tether_length", 1.50)      # [m] Lunghezza TOTALE nastro
        self.declare_parameter("pole_height", 0.95)        # [m] Altezza pali sopra i robot
        
        # Target height control
        self.declare_parameter("target_object_height", 0.30)   # [m] Altezza desiderata oggetto in LIFTING
        self.declare_parameter("height_tolerance", 0.03)       # [m] Tolleranza per considerare altezza raggiunta
        self.declare_parameter("use_height_control", True)     # Se True, calcola distanza da altezza; altrimenti usa distanza fissa
        
        # Height feedback (closed-loop correction in LIFTING)
        self.declare_parameter("enable_height_feedback", False)
        self.declare_parameter("height_feedback_kp", 0.9)
        self.declare_parameter("height_feedback_max_delta_z", 0.12)  # [m]
        
        # Fallback: distanza fissa se use_height_control=False
        self.declare_parameter("lifting_horizontal_distance_fallback", 1.20)  # [m]
        
        # Transition timing (OTTIMIZZATO per rientro veloce)
        self.declare_parameter("approach_transition_time", 1.5)   # [s] NORMAL → APPROACHING
        self.declare_parameter("lifting_transition_time", 3.0)    # [s] ⚡ RIDOTTO: da 2.5 a 2.0
        self.declare_parameter("clearing_transition_time", 2.0)   # [s] ⚡ RIDOTTO: da 3.5 a 2.0 (rientro veloce!)
        
        # Transition progress can optionally react to obstacle distance (not only time)
        self.declare_parameter("use_distance_based_approach_progress", True)
        
        # Obstacle detection with ROBUST HYSTERESIS
        self.declare_parameter("enable_obstacle_detection", True)
        self.declare_parameter("costmap_topics", [
            "/tb3_3/local_costmap/costmap", 
            "/tb3_4/local_costmap/costmap"
        ])
        self.declare_parameter("obstacle_threshold", 20)
        
        # Filtro direzionale: considera ostacoli solo DAVANTI ai robot
        self.declare_parameter("enable_directional_detection", True)   # Rileva solo ostacoli nella direzione di movimento
        self.declare_parameter("detection_cone_angle", 120.0)           # [deg] Angolo cono davanti al robot (90° = ±45°)

        # Early verification before APPROACHING (optional)
        self.declare_parameter("obstacle_precheck_distance", 1.50)     # [m] Start confirming obstacle earlier than approach_distance

        # Ignore formation robots when scanning costmaps (reduce false positives)
        self.declare_parameter("ignore_formation_robots_as_obstacles", True)
        self.declare_parameter("formation_robot_ignore_radius", 0.55)  # [m] Should be >= robot radius + inflation radius

        # CLEARING re-lift protections
        self.declare_parameter("clearing_relift_cooldown_s", 0.8)      # [s] Ignore re-trigger into LIFTING shortly after entering CLEARING
        self.declare_parameter("retrigger_requires_decreasing_distance", True)
        self.declare_parameter("retrigger_delta_m", 0.05)              # [m] Minimum decrease to consider obstacle "approaching"
        
        # ISTERESI ROBUSTA (chiave per evitare comportamenti ambigui!)
        # LOGICA CORRETTA:
        #   approach_distance > activate_distance > clear_distance
        #   
        #   Ostacolo si avvicina:
        #     - A 0.80m: NORMAL → APPROACHING (inizia preparazione)
        #     - A 0.50m: APPROACHING → LIFTING (lifting completo)
        #   
        #   Ostacolo si allontana (OTTIMIZZATO per rientro veloce):
        #     - A 0.65m: LIFTING → CLEARING (inizia rientro PRIMA)
        #     - A 0.85m: CLEARING → NORMAL (formazione normale PRIMA)
        
        self.declare_parameter("obstacle_approach_distance", 0.80)     # [m] Inizia APPROACHING
        self.declare_parameter("obstacle_activate_distance", 0.50)     # [m] Attiva LIFTING completo
        self.declare_parameter("obstacle_clearing_distance", 0.55)     # [m] ⚡ RIDOTTO: inizia rientro appena possibile
        self.declare_parameter("obstacle_clear_distance", 0.65)        # [m] ⚡ RIDOTTO: torna a normale più velocemente
        
        self.declare_parameter("obstacle_check_radius", 0.80)          # [m] Raggio controllo ostacoli
        self.declare_parameter("obstacle_detection_count", 2)          # Conferme consecutive per attivare
        self.declare_parameter("obstacle_clear_count", 10)              # ⚡ RIDOTTO: da 8 a 5 (rientro più reattivo)
        
        # Optional time-based hysteresis (seconds). Use -1.0 to auto-compute from counts and loop rate.
        # Set to 0.0 to disable time-based hysteresis and rely on counts only.
        self.declare_parameter("obstacle_confirm_time", -1.0)
        self.declare_parameter("obstacle_clear_time_s", -1.0)
        
        # Safety limits (always enforced)
        self.declare_parameter("absolute_max_distance", 1.48)   # [m] Mai superare (sicurezza corda)
        self.declare_parameter("absolute_min_distance", 0.20)   # [m] Mai andare sotto
        
        # Formation setup mode
        self.declare_parameter("enable_setup_mode", True)
        self.declare_parameter("setup_max_distance", 2.5)
        self.declare_parameter("auto_detect_formation", True)
        self.declare_parameter("formation_threshold", 0.20)
        self.declare_parameter("formation_stable_count", 20)
        
        # Control loop rate
        self.declare_parameter("rate_hz", 30.0)
        
        # Debug
        self.declare_parameter("enable_debug_logs", True)
        
        # ==================== READ PARAMETERS ====================
        
        self.robot_namespaces = list(self.get_parameter("robot_namespaces").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        
        self.tether_length = float(self.get_parameter("tether_length").value)
        self.pole_height = float(self.get_parameter("pole_height").value)
        
        self.target_object_height = float(self.get_parameter("target_object_height").value)
        self.height_tolerance = float(self.get_parameter("height_tolerance").value)
        self.use_height_control = bool(self.get_parameter("use_height_control").value)
        self.enable_height_feedback = bool(self.get_parameter("enable_height_feedback").value)
        self.height_feedback_kp = float(self.get_parameter("height_feedback_kp").value)
        self.height_feedback_max_delta_z = float(self.get_parameter("height_feedback_max_delta_z").value)
        
        self.lifting_h_dist_fallback = float(self.get_parameter("lifting_horizontal_distance_fallback").value)
        
        self.approach_transition_time = float(self.get_parameter("approach_transition_time").value)
        self.lifting_transition_time = float(self.get_parameter("lifting_transition_time").value)
        self.clearing_transition_time = float(self.get_parameter("clearing_transition_time").value)
        self.use_distance_based_approach_progress = bool(self.get_parameter("use_distance_based_approach_progress").value)
        
        self.enable_obstacle_detection = bool(self.get_parameter("enable_obstacle_detection").value)
        self.costmap_topics = list(self.get_parameter("costmap_topics").value)
        self.obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)
        
        self.enable_directional_detection = bool(self.get_parameter("enable_directional_detection").value)
        self.detection_cone_angle = float(self.get_parameter("detection_cone_angle").value)
        self.obstacle_precheck_distance = float(self.get_parameter("obstacle_precheck_distance").value)
        self.ignore_formation_robots_as_obstacles = bool(self.get_parameter("ignore_formation_robots_as_obstacles").value)
        self.formation_robot_ignore_radius = float(self.get_parameter("formation_robot_ignore_radius").value)
        self.clearing_relift_cooldown_s = float(self.get_parameter("clearing_relift_cooldown_s").value)
        self.retrigger_requires_decreasing_distance = bool(self.get_parameter("retrigger_requires_decreasing_distance").value)
        self.retrigger_delta_m = float(self.get_parameter("retrigger_delta_m").value)

        
        self.obstacle_approach_distance = float(self.get_parameter("obstacle_approach_distance").value)
        self.obstacle_activate_distance = float(self.get_parameter("obstacle_activate_distance").value)
        self.obstacle_clearing_distance = float(self.get_parameter("obstacle_clearing_distance").value)
        self.obstacle_clear_distance = float(self.get_parameter("obstacle_clear_distance").value)
        self.obstacle_check_radius = float(self.get_parameter("obstacle_check_radius").value)
        self.obstacle_detection_count = int(self.get_parameter("obstacle_detection_count").value)
        self.obstacle_clear_count = int(self.get_parameter("obstacle_clear_count").value)
        self.obstacle_confirm_time = float(self.get_parameter("obstacle_confirm_time").value)
        self.obstacle_clear_time_s = float(self.get_parameter("obstacle_clear_time_s").value)
        
        self.absolute_max_distance = float(self.get_parameter("absolute_max_distance").value)
        self.absolute_min_distance = float(self.get_parameter("absolute_min_distance").value)
        
        self.enable_setup_mode = bool(self.get_parameter("enable_setup_mode").value)
        self.setup_max_distance = float(self.get_parameter("setup_max_distance").value)
        self.auto_detect_formation = bool(self.get_parameter("auto_detect_formation").value)
        self.formation_threshold = float(self.get_parameter("formation_threshold").value)
        self.formation_stable_count = int(self.get_parameter("formation_stable_count").value)
        
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.enable_debug_logs = bool(self.get_parameter("enable_debug_logs").value)
        
        # Auto-compute time-based hysteresis if requested
        if self.obstacle_confirm_time < 0.0:
            self.obstacle_confirm_time = max(0.0, self.obstacle_detection_count / max(self.rate_hz, 1e-6))
        if self.obstacle_clear_time_s < 0.0:
            self.obstacle_clear_time_s = max(0.0, self.obstacle_clear_count / max(self.rate_hz, 1e-6))
        
        # ==================== VALIDATE PARAMETERS ====================
        
        # Validate hysteresis configuration (CRITICO!)
        if not (self.obstacle_activate_distance < self.obstacle_approach_distance):
            self.get_logger().error(
                f"❌ ERRORE ISTERESI: obstacle_activate_distance ({self.obstacle_activate_distance}) "
                f"deve essere < obstacle_approach_distance ({self.obstacle_approach_distance})"
            )
            raise ValueError("Invalid hysteresis: activate >= approach")
        
        if not (self.obstacle_clearing_distance < self.obstacle_clear_distance):
            self.get_logger().error(
                f"❌ ERRORE ISTERESI: obstacle_clearing_distance ({self.obstacle_clearing_distance}) "
                f"deve essere < obstacle_clear_distance ({self.obstacle_clear_distance})"
            )
            raise ValueError("Invalid hysteresis: clearing >= clear")

        # Validate precheck distance
        if self.obstacle_precheck_distance < self.obstacle_approach_distance:
            self.get_logger().warn(
                f"⚠️ obstacle_precheck_distance ({self.obstacle_precheck_distance:.2f}m) < obstacle_approach_distance "
                f"({self.obstacle_approach_distance:.2f}m). Clamping to approach_distance."
            )
            self.obstacle_precheck_distance = self.obstacle_approach_distance

        # Validate ignore radius
        if self.formation_robot_ignore_radius < 0.0:
            self.get_logger().warn(
                f"⚠️ formation_robot_ignore_radius is negative ({self.formation_robot_ignore_radius:.2f}). Setting to 0.0."
            )
            self.formation_robot_ignore_radius = 0.0

        if self.clearing_relift_cooldown_s < 0.0:
            self.get_logger().warn(
                f"⚠️ clearing_relift_cooldown_s is negative ({self.clearing_relift_cooldown_s:.2f}). Setting to 0.0."
            )
            self.clearing_relift_cooldown_s = 0.0

        if self.retrigger_delta_m < 0.0:
            self.get_logger().warn(
                f"⚠️ retrigger_delta_m is negative ({self.retrigger_delta_m:.2f}). Setting to 0.0."
            )
            self.retrigger_delta_m = 0.0
        # Validate target height is achievable (also considering safety max distance)
        max_possible_height = self.pole_height
        max_height_safe = self._calculate_object_height(self.absolute_max_distance)
        hard_max_height = min(max_possible_height, max_height_safe)
        if self.target_object_height > hard_max_height:
            self.get_logger().warn(
                f"⚠️ Target height {self.target_object_height:.3f}m exceeds max achievable {hard_max_height:.3f}m "
                f"(pole={max_possible_height:.3f}m, safety@dmax={max_height_safe:.3f}m). Clamping."
            )
            self.target_object_height = hard_max_height * 0.98  # Leave margin

        # Calculate target lifting distance from desired height
        if self.use_height_control:
            self.lifting_h_dist = self._calculate_horizontal_distance(self.target_object_height)
            # Enforce safety bounds also on the computed lifting distance
            self.lifting_h_dist = max(self.absolute_min_distance, min(self.absolute_max_distance, self.lifting_h_dist))
            calc_check = self._calculate_object_height(self.lifting_h_dist)
            self.get_logger().info(
                f"🎯 Height control ENABLED:\n"
                f"   Target height: {self.target_object_height:.3f}m\n"
                f"   → Requires horizontal distance: {self.lifting_h_dist:.3f}m\n"
                f"   → Verification: calculated height = {calc_check:.3f}m"
            )
        else:
            self.lifting_h_dist = self.lifting_h_dist_fallback
            # Enforce safety bounds also on the fixed lifting distance
            self.lifting_h_dist = max(self.absolute_min_distance, min(self.absolute_max_distance, self.lifting_h_dist))
            calc_height = self._calculate_object_height(self.lifting_h_dist)
            self.get_logger().info(
                f"📏 Using FIXED distance mode:\n"
                f"   Horizontal distance: {self.lifting_h_dist:.3f}m\n"
                f"   → Results in object height: {calc_height:.3f}m"
            )
        
        # ==================== STATE VARIABLES ====================
        
        # Current operation mode
        self.current_mode = OperationMode.NORMAL
        self.mode_start_time: Optional[Time] = None
        
        # Track goals from formation_follower
        self.robot_goals: Dict[str, RobotGoalState] = {
            ns: RobotGoalState() for ns in self.robot_namespaces
        }
        
        # Formation status
        self.in_formation = False
        self.formation_check_count = 0
        
        # Obstacle detection state
        self.costmaps: Dict[str, Optional[OccupancyGrid]] = {
            ns: None for ns in self.robot_namespaces
        }
        self.obstacle_state = ObstacleState()
        self._last_real_distance: Optional[float] = None  # Used for height feedback
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ==================== SUBSCRIBERS ====================
        
        # Subscribe to ideal goals from formation_follower
        self.goal_subscribers = {}
        for ns in self.robot_namespaces:
            topic = f"/{ns}/formation_goal"
            self.goal_subscribers[ns] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, robot_ns=ns: self._goal_callback(msg, robot_ns),
                10
            )
            self.get_logger().info(f"📥 Subscribing to: {topic}")
        
        # Subscribe to costmaps
        if self.enable_obstacle_detection:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
            
            self.costmap_subscribers = {}
            for i, topic in enumerate(self.costmap_topics):
                ns = self.robot_namespaces[i] if i < len(self.robot_namespaces) else f"robot_{i}"
                self.costmap_subscribers[ns] = self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, robot_ns=ns: self._costmap_callback(msg, robot_ns),
                    qos_profile
                )
                self.get_logger().info(f"📥 Subscribing to costmap: {topic}")
        
        # ==================== PUBLISHERS ====================
        
        # Publish modified goals (with tether constraints applied)
        self.goal_publishers = {}
        for ns in self.robot_namespaces:
            topic = f"/{ns}/goal_update"
            self.goal_publishers[ns] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"📤 Publishing modified goals to: {topic}")
        
        # Status/debug publishers
        self.mode_pub = self.create_publisher(String, "/tether/mode", 10)
        self.formation_status_pub = self.create_publisher(Bool, "/formation/status", 10)
        self.real_distance_pub = self.create_publisher(Float32, "/tether/distance/real", 10)
        self.tether_distance_pub = self.create_publisher(Float32, "/tether/distance/goal", 10)
        self.object_height_pub = self.create_publisher(Float32, "/tether/object_height", 10)
        self.target_distance_pub = self.create_publisher(Float32, "/tether/distance/target", 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, "/tether/obstacle_distance", 10)  # DEBUG
        
        # ==================== TIMER ====================
        
        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._process_goals)
        
        # ==================== STARTUP INFO ====================
        
        self.get_logger().info(
            "="*70 + "\n" +
            "🚀 Tether Constraint Manager OPTIMIZED - STARTED\n" +
            "="*70 + "\n" +
            f"Physical Setup:\n" +
            f"  Tether length: {self.tether_length:.2f}m\n" +
            f"  Pole height: {self.pole_height:.2f}m\n" +
            f"  Height control: {'ENABLED' if self.use_height_control else 'DISABLED'}\n" +
            f"  Target lifting distance: {self.lifting_h_dist:.3f}m\n" +
            f"\n" +
            f"Hysteresis Configuration (CRITICAL!):\n" +
            f"  Obstacle approach: {self.obstacle_approach_distance:.2f}m → APPROACHING mode\n" +
            f"  Obstacle activate: {self.obstacle_activate_distance:.2f}m → LIFTING mode\n" +
            f"  Obstacle clearing: {self.obstacle_clearing_distance:.2f}m → CLEARING mode\n" +
            f"  Obstacle clear: {self.obstacle_clear_distance:.2f}m → NORMAL mode\n" +
            f"  Detection count: {self.obstacle_detection_count} confirms to activate\n" +
            f"  Clear count: {self.obstacle_clear_count} confirms to deactivate\n" +
            f"\n" +
            f"Transition Times:\n" +
            f"  Approach: {self.approach_transition_time:.1f}s\n" +
            f"  Lifting: {self.lifting_transition_time:.1f}s\n" +
            f"  Clearing: {self.clearing_transition_time:.1f}s\n" +
            "="*70
        )
    
    # ==================== CALLBACKS ====================
    
    def _goal_callback(self, msg: PoseStamped, robot_ns: str):
        """Store incoming formation goal"""
        self.robot_goals[robot_ns].goal = msg
        self.robot_goals[robot_ns].last_update_time = self.get_clock().now()
    
    def _costmap_callback(self, msg: OccupancyGrid, robot_ns: str):
        """Store incoming costmap"""
        self.costmaps[robot_ns] = msg
    
    # ==================== GEOMETRY CALCULATIONS ====================
    
    def _calculate_object_height(self, horizontal_distance: float) -> float:
        """
        Calculate object height from horizontal distance between robots.
        
        Geometry:
        - Pole height: h_pole
        - Tether length: L (total)
        - Each side from pole to object: L/2
        - Horizontal distance between robots: d
        - Each side horizontal projection: d/2
        
        Using Pythagoras on one half:
           (L/2)² = (d/2)² + (h_pole - z)²
        
        Solving for z:
           z = h_pole - sqrt((L/2)² - (d/2)²)
        """
        half_tether = self.tether_length / 2.0
        half_dist = horizontal_distance / 2.0
        
        # Safety: cannot exceed tether length
        if half_dist > half_tether:
            return 0.0  # Object on ground (overstretched)
        
        vertical_drop = math.sqrt(half_tether**2 - half_dist**2)
        object_height = self.pole_height - vertical_drop
        
        return max(0.0, object_height)
    
    def _calculate_horizontal_distance(self, target_height: float) -> float:
        """
        Calculate required horizontal distance to achieve target object height.
        
        Inverse of _calculate_object_height:
           z = h_pole - sqrt((L/2)² - (d/2)²)
           
        Solving for d:
           sqrt((L/2)² - (d/2)²) = h_pole - z
           (L/2)² - (d/2)² = (h_pole - z)²
           (d/2)² = (L/2)² - (h_pole - z)²
           d = 2 * sqrt((L/2)² - (h_pole - z)²)
        """
        half_tether = self.tether_length / 2.0
        vertical_drop = self.pole_height - target_height
        
        # Safety checks
        if vertical_drop < 0:
            self.get_logger().warn("Target height exceeds pole height!")
            return 0.0
        
        if vertical_drop > half_tether:
            self.get_logger().warn("Target height requires tether longer than available!")
            return 0.0
        
        half_dist = math.sqrt(half_tether**2 - vertical_drop**2)
        horizontal_distance = 2.0 * half_dist
        
        return horizontal_distance
    # ==================== TF & POSITION ====================

    @staticmethod
    def _quat_mul(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """Quaternion multiplication (x, y, z, w)."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """Quaternion conjugate (x, y, z, w)."""
        x, y, z, w = q
        return (-x, -y, -z, w)

    @classmethod
    def _quat_rotate_vector(
        cls,
        q: Tuple[float, float, float, float],
        v: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """Rotate vector v by quaternion q."""
        vx, vy, vz = v
        vq = (vx, vy, vz, 0.0)
        qvq = cls._quat_mul(cls._quat_mul(q, vq), cls._quat_conj(q))
        return (qvq[0], qvq[1], qvq[2])

    def _transform_point(
        self,
        x: float,
        y: float,
        z: float,
        from_frame: str,
        to_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        """Transform a 3D point from from_frame to to_frame using TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            q = (
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            )
            rx, ry, rz = self._quat_rotate_vector(q, (x, y, z))
            rx += tf.transform.translation.x
            ry += tf.transform.translation.y
            rz += tf.transform.translation.z
            return rx, ry, rz
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.enable_debug_logs:
                self.get_logger().warn(
                    f"TF point transform failed {from_frame} -> {to_frame}: {e}",
                    throttle_duration_sec=2.0,
                )
            return None

    def _get_robot_pose_in_frame(self, robot_ns: str, target_frame: str) -> Optional[Tuple[float, float, float]]:
        """Get robot pose (x, y, yaw) expressed in target_frame."""
        source_frame = f"{robot_ns}/{self.base_frame}"
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = yaw_from_quaternion(transform.transform.rotation)
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.enable_debug_logs:
                self.get_logger().warn(
                    f"TF lookup failed for {source_frame} -> {target_frame}: {e}",
                    throttle_duration_sec=2.0,
                )
            return None

    def _get_robot_position(self, robot_ns: str) -> Optional[Tuple[float, float, float]]:
        """Get robot position (x, y, yaw) in the configured global frame."""
        return self._get_robot_pose_in_frame(robot_ns, self.global_frame)

    def _get_goal_xy_in_frame(self, goal: PoseStamped, target_frame: str) -> Optional[Tuple[float, float]]:
        """Return goal (x, y) expressed in target_frame."""
        goal_frame = goal.header.frame_id.strip() if goal.header.frame_id else self.global_frame
        if goal_frame == target_frame:
            return goal.pose.position.x, goal.pose.position.y

        pt = self._transform_point(
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
            goal_frame,
            target_frame,
        )
        if pt is None:
            return None
        return pt[0], pt[1]

    def _get_motion_yaw(
        self,
        robot_ns: str,
        rx: float,
        ry: float,
        fallback_yaw: float,
        frame_id: str,
    ) -> float:
        """Estimate motion direction yaw using the current goal (fallback to robot yaw)."""
        goal_state = self.robot_goals.get(robot_ns)
        if goal_state is None or goal_state.goal is None:
            return fallback_yaw

        goal_xy = self._get_goal_xy_in_frame(goal_state.goal, frame_id)
        if goal_xy is None:
            return fallback_yaw

        gx, gy = goal_xy
        return math.atan2(gy - ry, gx - rx)

    @staticmethod
    def _compute_distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """Compute Euclidean distance"""
        return math.hypot(x2 - x1, y2 - y1)
    
    # ==================== OBSTACLE DETECTION ====================
    
    def _detect_obstacles_in_costmaps(self) -> float:
        """
        Check for obstacles in costmaps.
        Returns closest obstacle distance, or inf if none found.
        
        MIGLIORAMENTO: Considera solo ostacoli nella direzione di movimento (cono davanti)
        per evitare falsi positivi da ostacoli già superati.
        """
        closest_dist = float('inf')
        
        for ns, costmap in self.costmaps.items():
            if costmap is None:
                continue
            
            # Get robot pose in the costmap frame (costmap origin is expressed in this frame)
            costmap_frame = costmap.header.frame_id.strip() if costmap.header.frame_id else self.global_frame
            robot_pos = self._get_robot_pose_in_frame(ns, costmap_frame)
            if robot_pos is None:
                continue
            
            rx, ry, robot_yaw = robot_pos
            motion_yaw = self._get_motion_yaw(ns, rx, ry, robot_yaw, costmap_frame)

            # Optionally ignore other formation robots (avoid detecting teammates as obstacles)
            ignore_centers = []
            if self.ignore_formation_robots_as_obstacles:
                for other_ns in self.robot_namespaces:
                    if other_ns == ns:
                        continue
                    other_pose = self._get_robot_pose_in_frame(other_ns, costmap_frame)
                    if other_pose is None:
                        continue
                    ignore_centers.append((other_pose[0], other_pose[1]))

            
            # Convert robot position to costmap coordinates
            resolution = costmap.info.resolution
            origin_x = costmap.info.origin.position.x
            origin_y = costmap.info.origin.position.y
            width = costmap.info.width
            height = costmap.info.height
            
            # Robot position in costmap grid
            robot_cx = int((rx - origin_x) / resolution)
            robot_cy = int((ry - origin_y) / resolution)
            
            # Check radius in grid cells
            check_radius_cells = int(self.obstacle_check_radius / resolution)
            
            # Scan area around robot
            for dy in range(-check_radius_cells, check_radius_cells + 1):
                for dx in range(-check_radius_cells, check_radius_cells + 1):
                    cx = robot_cx + dx
                    cy = robot_cy + dy
                    
                    # Bounds check
                    if cx < 0 or cx >= width or cy < 0 or cy >= height:
                        continue
                    
                    # Get costmap value
                    index = cy * width + cx
                    if index >= len(costmap.data):
                        continue
                    
                    cost = costmap.data[index]
                    
                    # Check if this is an obstacle
                    if cost >= self.obstacle_threshold:
                        # Calculate position and distance
                        obs_x = origin_x + cx * resolution
                        obs_y = origin_y + cy * resolution
                        dist = self._compute_distance(rx, ry, obs_x, obs_y)

                        # Ignore cells close to other formation robots (covers inflation around them)
                        if ignore_centers and self.formation_robot_ignore_radius > 0.0:
                            near_teammate = False
                            for ox, oy in ignore_centers:
                                if self._compute_distance(obs_x, obs_y, ox, oy) <= self.formation_robot_ignore_radius:
                                    near_teammate = True
                                    break
                            if near_teammate:
                                continue

                        
                        # FILTRO DIREZIONALE: considera solo ostacoli davanti
                        if self.enable_directional_detection:
                            # Angolo dall'asse del robot all'ostacolo
                            angle_to_obstacle = math.atan2(obs_y - ry, obs_x - rx)
                            
                            # Differenza angolare rispetto alla direzione del robot
                            angle_diff = abs(self._normalize_angle(angle_to_obstacle - motion_yaw))
                            
                            # Converti detection_cone_angle da gradi totali a radianti per metà cono
                            half_cone_rad = math.radians(self.detection_cone_angle / 2.0)
                            
                            # Se ostacolo fuori dal cono davanti, ignora
                            if angle_diff > half_cone_rad:
                                continue
                        
                        closest_dist = min(closest_dist, dist)
        
        return closest_dist
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def _update_obstacle_detection(self):
        """
        Update obstacle detection state with hysteresis.

        State machine con isteresi robusta:
        - NORMAL → APPROACHING: quando ostacolo < approach_distance (con conferma count o tempo)
        - APPROACHING → LIFTING: quando ostacolo < activate_distance (con conferma count o tempo)
        - LIFTING → CLEARING: quando ostacolo > clearing_distance (con conferma count o tempo)
        - CLEARING → NORMAL: quando ostacolo > clear_distance (con conferma count o tempo)
        """
        if not self.enable_obstacle_detection:
            return

        # Do not detect obstacles during initial setup (avoid conflicts while forming)
        if self.enable_setup_mode and not self.in_formation:
            return

        closest_obstacle = self._detect_obstacles_in_costmaps()
        self.obstacle_state.closest_distance = closest_obstacle

        # Publish for debug (-1 means "none")
        obs_msg = Float32()
        obs_msg.data = closest_obstacle if closest_obstacle != float("inf") else -1.0
        self.obstacle_distance_pub.publish(obs_msg)

        now = self.get_clock().now()

        def elapsed_s(t0: Optional[Time]) -> float:
            if t0 is None:
                return 0.0
            return (now - t0).nanoseconds / 1e9

        use_time_confirm = self.obstacle_confirm_time > 0.0
        use_time_clear = self.obstacle_clear_time_s > 0.0

        current_mode = self.current_mode

        # Track distance trend (used to prevent re-trigger into LIFTING due to noise/behind obstacles)
        prev_dist = self.obstacle_state.prev_closest_distance
        self.obstacle_state.prev_closest_distance = closest_obstacle

        obstacle_approaching = True
        if self.retrigger_requires_decreasing_distance:
            if closest_obstacle == float("inf"):
                obstacle_approaching = False
            elif prev_dist == float("inf"):
                obstacle_approaching = True
            else:
                obstacle_approaching = (prev_dist - closest_obstacle) >= self.retrigger_delta_m

        # Early verification: confirm obstacle earlier than approach_distance, without changing mode
        if use_time_confirm and (self.obstacle_precheck_distance > self.obstacle_approach_distance):
            if closest_obstacle < self.obstacle_precheck_distance:
                if self.obstacle_state.precheck_enter_time is None:
                    self.obstacle_state.precheck_enter_time = now
                if elapsed_s(self.obstacle_state.precheck_enter_time) >= self.obstacle_confirm_time:
                    self.obstacle_state.prechecked = True
            else:
                self.obstacle_state.precheck_enter_time = None
                self.obstacle_state.prechecked = False
        else:
            self.obstacle_state.precheck_enter_time = None
            self.obstacle_state.prechecked = False


        # === NORMAL → APPROACHING ===
        if current_mode == OperationMode.NORMAL:
            if closest_obstacle < self.obstacle_approach_distance:
                # If obstacle was already confirmed during pre-check, transition immediately
                if self.obstacle_state.prechecked:
                    self.obstacle_state.prechecked = False
                    self.obstacle_state.precheck_enter_time = None
                    self.obstacle_state.detect_enter_time = None
                    self.obstacle_state.detection_count = 0
                    self._transition_to_mode(OperationMode.APPROACHING)
                    self.get_logger().info(
                        f"🔶 Obstacle pre-confirmed at {closest_obstacle:.2f}m → APPROACHING mode"
                    )
                    return

                self.obstacle_state.clear_count = 0
                self.obstacle_state.clear_enter_time = None

                if use_time_confirm:
                    if self.obstacle_state.detect_enter_time is None:
                        self.obstacle_state.detect_enter_time = now
                    if elapsed_s(self.obstacle_state.detect_enter_time) >= self.obstacle_confirm_time:
                        self.obstacle_state.detect_enter_time = None
                        self.obstacle_state.detection_count = 0
                        self._transition_to_mode(OperationMode.APPROACHING)
                        self.get_logger().info(
                            f"🔶 Obstacle detected at {closest_obstacle:.2f}m → APPROACHING mode"
                        )
                else:
                    self.obstacle_state.detection_count += 1
                    if self.obstacle_state.detection_count >= self.obstacle_detection_count:
                        self.obstacle_state.detection_count = 0
                        self._transition_to_mode(OperationMode.APPROACHING)
                        self.get_logger().info(
                            f"🔶 Obstacle detected at {closest_obstacle:.2f}m → APPROACHING mode"
                        )
            else:
                self.obstacle_state.detection_count = 0
                self.obstacle_state.detect_enter_time = None

        # === APPROACHING → LIFTING (or back to NORMAL) ===
        elif current_mode == OperationMode.APPROACHING:
            # HARD TRIGGER: immediate lifting when obstacle enters critical distance
            if closest_obstacle < self.obstacle_activate_distance:
                self._transition_to_mode(OperationMode.LIFTING)
                self.get_logger().warn(
                    f"🚨 HARD LIFT TRIGGER at {closest_obstacle:.2f}m"
                )
                return

            elif closest_obstacle == float("inf") or closest_obstacle > self.obstacle_clear_distance:
                # Obstacle disappeared before full lift - go back to NORMAL
                self.obstacle_state.detection_count = 0
                self.obstacle_state.detect_enter_time = None

                if use_time_clear:
                    if self.obstacle_state.clear_enter_time is None:
                        self.obstacle_state.clear_enter_time = now
                    if elapsed_s(self.obstacle_state.clear_enter_time) >= self.obstacle_clear_time_s:
                        self.obstacle_state.clear_enter_time = None
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.NORMAL)
                        self.get_logger().info(
                            f"✅ Obstacle cleared → NORMAL mode"
                        )
                else:
                    self.obstacle_state.clear_count += 1
                    if self.obstacle_state.clear_count >= self.obstacle_clear_count:
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.NORMAL)
                        self.get_logger().info(
                            f"✅ Obstacle cleared → NORMAL mode"
                        )
            else:
                # Stay in APPROACHING
                self.obstacle_state.detection_count = 0
                self.obstacle_state.clear_count = 0
                self.obstacle_state.detect_enter_time = None
                self.obstacle_state.clear_enter_time = None

        # === LIFTING → CLEARING ===
        elif current_mode == OperationMode.LIFTING:
            if closest_obstacle == float("inf") or closest_obstacle > self.obstacle_clearing_distance:
                self.obstacle_state.detection_count = 0
                self.obstacle_state.detect_enter_time = None

                if use_time_clear:
                    if self.obstacle_state.clear_enter_time is None:
                        self.obstacle_state.clear_enter_time = now
                    if elapsed_s(self.obstacle_state.clear_enter_time) >= self.obstacle_clear_time_s:
                        self.obstacle_state.clear_enter_time = None
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.CLEARING)
                        self.get_logger().info(
                            f"🔽 Obstacle passed → CLEARING mode"
                        )
                else:
                    self.obstacle_state.clear_count += 1
                    if self.obstacle_state.clear_count >= self.obstacle_clear_count:
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.CLEARING)
                        self.get_logger().info(
                            f"🔽 Obstacle passed → CLEARING mode"
                        )
            else:
                # Still close, stay in LIFTING
                self.obstacle_state.clear_count = 0
                self.obstacle_state.clear_enter_time = None

        # === CLEARING → NORMAL (or back to LIFTING) ===
        elif current_mode == OperationMode.CLEARING:
            # Enforce minimum clearing time to avoid bouncing
            if elapsed_s(self.mode_start_time) < self.clearing_transition_time:
                return

            if (
                (closest_obstacle == float("inf") or closest_obstacle > self.obstacle_clear_distance)
                and not obstacle_approaching
            ):
                self.obstacle_state.detection_count = 0
                self.obstacle_state.detect_enter_time = None

                if use_time_clear:
                    if self.obstacle_state.clear_enter_time is None:
                        self.obstacle_state.clear_enter_time = now
                    if elapsed_s(self.obstacle_state.clear_enter_time) >= self.obstacle_clear_time_s:
                        self.obstacle_state.clear_enter_time = None
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.NORMAL)
                        self.get_logger().info(
                            f"✅ Obstacle fully cleared → NORMAL mode"
                        )
                else:
                    self.obstacle_state.clear_count += 1
                    if self.obstacle_state.clear_count >= self.obstacle_clear_count:
                        self.obstacle_state.clear_count = 0
                        self._transition_to_mode(OperationMode.NORMAL)
                        self.get_logger().info(
                            f"✅ Obstacle fully cleared → NORMAL mode"
                        )

            # Prevent immediate re-trigger into LIFTING after entering CLEARING
            cooldown_active = False
            if self.obstacle_state.clearing_mode_enter_time is not None:
                cooldown_active = elapsed_s(self.obstacle_state.clearing_mode_enter_time) < self.clearing_relift_cooldown_s

            elif closest_obstacle < self.obstacle_activate_distance:
                # Obstacle came back - optionally return to LIFTING (with protections)
                if cooldown_active:
                    # Stay in CLEARING during cooldown
                    self.obstacle_state.detection_count = 0
                    self.obstacle_state.detect_enter_time = None
                    return

                if self.retrigger_requires_decreasing_distance and (not obstacle_approaching):
                    # Do not re-trigger if obstacle is not getting closer (often behind or noise)
                    self.obstacle_state.detection_count = 0
                    self.obstacle_state.detect_enter_time = None
                    return

                self.obstacle_state.clear_count = 0
                self.obstacle_state.clear_enter_time = None

                if use_time_confirm:
                    if self.obstacle_state.detect_enter_time is None:
                        self.obstacle_state.detect_enter_time = now
                    if elapsed_s(self.obstacle_state.detect_enter_time) >= self.obstacle_confirm_time:
                        self.obstacle_state.detect_enter_time = None
                        self.obstacle_state.detection_count = 0
                        self._transition_to_mode(OperationMode.LIFTING)
                        self.get_logger().warn(
                            f"⚠️ Obstacle returned at {closest_obstacle:.2f}m → LIFTING mode"
                        )
                else:
                    self.obstacle_state.detection_count += 1
                    if self.obstacle_state.detection_count >= self.obstacle_detection_count:
                        self.obstacle_state.detection_count = 0
                        self._transition_to_mode(OperationMode.LIFTING)
                        self.get_logger().warn(
                            f"⚠️ Obstacle returned at {closest_obstacle:.2f}m → LIFTING mode"
                        )
            else:
                # Stay in CLEARING
                self.obstacle_state.detection_count = 0
                self.obstacle_state.clear_count = 0
                self.obstacle_state.detect_enter_time = None
                self.obstacle_state.clear_enter_time = None

    # ==================== MODE TRANSITIONS ====================

    
    def _transition_to_mode(self, new_mode: OperationMode):
        """Transition to a new operation mode"""
        if new_mode != self.current_mode:
            old_mode_name = self.current_mode.name
            self.current_mode = new_mode
            self.mode_start_time = self.get_clock().now()
            # Reset obstacle pre-check when mode changes
            self.obstacle_state.prechecked = False
            self.obstacle_state.precheck_enter_time = None
            self.obstacle_state.detect_enter_time = None
            self.obstacle_state.clear_enter_time = None
            self.obstacle_state.detection_count = 0
            self.obstacle_state.clear_count = 0

            # Track entry time into CLEARING to prevent immediate re-trigger into LIFTING
            if new_mode == OperationMode.CLEARING:
                self.obstacle_state.clearing_mode_enter_time = self.mode_start_time
            elif new_mode == OperationMode.NORMAL:
                self.obstacle_state.clearing_mode_enter_time = None

            
            # Publish mode change
            mode_msg = String()
            mode_msg.data = new_mode.name
            self.mode_pub.publish(mode_msg)
            
            self.get_logger().info(
                f"🔄 MODE TRANSITION: {old_mode_name} → {new_mode.name}"
            )
    
    def _get_transition_progress(self) -> float:
        """
        Get transition progress [0.0, 1.0] based on current mode and time.
        
        Returns:
            0.0 = start of transition
            1.0 = fully transitioned
        """
        if self.mode_start_time is None:
            return 1.0
        
        elapsed = (self.get_clock().now() - self.mode_start_time).nanoseconds / 1e9
        
        if self.current_mode == OperationMode.APPROACHING:
            progress = min(1.0, elapsed / self.approach_transition_time)
        elif self.current_mode == OperationMode.LIFTING:
            progress = min(1.0, elapsed / self.lifting_transition_time)
        elif self.current_mode == OperationMode.CLEARING:
            progress = min(1.0, elapsed / self.clearing_transition_time)
        else:  # NORMAL
            progress = 1.0
        
        return progress
    
    # ==================== GOAL MODIFICATION ====================
    
    def _spread_goals_apart(
        self,
        goal1: PoseStamped,
        goal2: PoseStamped,
        target_distance: float
    ) -> Tuple[PoseStamped, PoseStamped]:
        """
        Modify goals to enforce a specific distance while maintaining:
        - Same midpoint
        - Same relative orientation
        - Same direction between robots
        """
        x1 = goal1.pose.position.x
        y1 = goal1.pose.position.y
        x2 = goal2.pose.position.x
        y2 = goal2.pose.position.y
        
        # Midpoint
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        
        # Direction from midpoint to each goal
        dx1 = x1 - mid_x
        dy1 = y1 - mid_y
        dx2 = x2 - mid_x
        dy2 = y2 - mid_y
        
        current_dist = math.hypot(dx1 - dx2, dy1 - dy2)
        if current_dist < 1e-6:
            # Goals coincide, spread them along x-axis
            new_goal1 = PoseStamped()
            new_goal1.header = goal1.header
            new_goal1.header.stamp = self.get_clock().now().to_msg()
            new_goal1.pose.position.x = mid_x - target_distance / 2.0
            new_goal1.pose.position.y = mid_y
            new_goal1.pose.position.z = 0.0
            new_goal1.pose.orientation = goal1.pose.orientation
            
            new_goal2 = PoseStamped()
            new_goal2.header = goal2.header
            new_goal2.header.stamp = self.get_clock().now().to_msg()
            new_goal2.pose.position.x = mid_x + target_distance / 2.0
            new_goal2.pose.position.y = mid_y
            new_goal2.pose.position.z = 0.0
            new_goal2.pose.orientation = goal2.pose.orientation
            
            return new_goal1, new_goal2
        
        # Scale to target distance
        scale = (target_distance / 2.0) / math.hypot(dx1, dy1)
        
        new_goal1 = PoseStamped()
        new_goal1.header = goal1.header
        new_goal1.header.stamp = self.get_clock().now().to_msg()
        new_goal1.pose.position.x = mid_x + dx1 * scale
        new_goal1.pose.position.y = mid_y + dy1 * scale
        new_goal1.pose.position.z = 0.0
        new_goal1.pose.orientation = goal1.pose.orientation
        
        new_goal2 = PoseStamped()
        new_goal2.header = goal2.header
        new_goal2.header.stamp = self.get_clock().now().to_msg()
        new_goal2.pose.position.x = mid_x + dx2 * scale
        new_goal2.pose.position.y = mid_y + dy2 * scale
        new_goal2.pose.position.z = 0.0
        new_goal2.pose.orientation = goal2.pose.orientation
        
        return new_goal1, new_goal2
    
    # ==================== FORMATION STATUS ====================
    
    def _check_formation_status(self):
        """Check if robots are in formation"""
        if not self.auto_detect_formation or self.in_formation:
            return
        
        all_in_position = True
        
        for ns in self.robot_namespaces:
            goal_state = self.robot_goals[ns]
            if goal_state.goal is None:
                all_in_position = False
                break
            
            pos = self._get_robot_position(ns)
            if pos is None:
                all_in_position = False
                break
            
            distance_to_goal = self._compute_distance(
                pos[0], pos[1],
                goal_state.goal.pose.position.x,
                goal_state.goal.pose.position.y
            )
            
            if distance_to_goal > self.formation_threshold:
                all_in_position = False
                break
        
        if all_in_position:
            self.formation_check_count += 1
            if self.formation_check_count >= self.formation_stable_count:
                if not self.in_formation:
                    self.get_logger().info("✅ Formation achieved!")
                    self.in_formation = True
                    msg = Bool()
                    msg.data = True
                    self.formation_status_pub.publish(msg)
        else:
            self.formation_check_count = 0
    
    # ==================== MAIN PROCESSING ====================
    
    def _process_goals(self):
        """Main processing loop"""
        
        # Update obstacle detection and mode transitions
        self._update_obstacle_detection()
        
        # Update formation status
        self._check_formation_status()
        
        # Check if we have goals for all robots
        if not all(state.goal is not None for state in self.robot_goals.values()):
            return
        
        # For 2 robots
        if len(self.robot_namespaces) != 2:
            self.get_logger().warn(
                "Tether constraint manager currently supports only 2 robots",
                throttle_duration_sec=5.0
            )
            return
        
        ns1, ns2 = self.robot_namespaces[0], self.robot_namespaces[1]
        
        goal1 = self.robot_goals[ns1].goal
        goal2 = self.robot_goals[ns2].goal
        
        pos1 = self._get_robot_position(ns1)
        pos2 = self._get_robot_position(ns2)
        
        if pos1 is None or pos2 is None:
            # No TF available, pass through
            self.goal_publishers[ns1].publish(goal1)
            self.goal_publishers[ns2].publish(goal2)
            return
        
        # Calculate distances
        real_distance = self._compute_distance(pos1[0], pos1[1], pos2[0], pos2[1])
        self._last_real_distance = real_distance
        goal_distance = self._compute_distance(
            goal1.pose.position.x, goal1.pose.position.y,
            goal2.pose.position.x, goal2.pose.position.y
        )
        
        # Publish metrics
        real_dist_msg = Float32()
        real_dist_msg.data = real_distance
        self.real_distance_pub.publish(real_dist_msg)
        
        goal_dist_msg = Float32()
        goal_dist_msg.data = goal_distance
        self.tether_distance_pub.publish(goal_dist_msg)
        
        # Calculate current object height
        current_z = self._calculate_object_height(real_distance)
        height_msg = Float32()
        height_msg.data = current_z
        self.object_height_pub.publish(height_msg)
        
        # Determine target distance based on mode
        target_dist = self._compute_target_distance(goal_distance)
        
        # Publish target distance
        target_dist_msg = Float32()
        target_dist_msg.data = target_dist
        self.target_distance_pub.publish(target_dist_msg)
        
        # === SETUP MODE: Durante formazione iniziale, permetti distanze ampie ===
        if self.enable_setup_mode and not self.in_formation:
            # Robot stanno ancora andando in formazione
            if goal_distance > self.setup_max_distance:
                self.get_logger().warn(
                    f"🔧 SETUP MODE: Goals too far ({goal_distance:.2f}m > {self.setup_max_distance:.2f}m)",
                    throttle_duration_sec=2.0
                )
                modified_goal1, modified_goal2 = self._spread_goals_apart(
                    goal1, goal2, self.setup_max_distance
                )
            elif goal_distance < self.absolute_min_distance:
                # Limite minimo sempre applicato
                self.get_logger().warn(
                    f"🔧 SETUP MODE: Goals too close ({goal_distance:.2f}m < {self.absolute_min_distance:.2f}m)",
                    throttle_duration_sec=2.0
                )
                modified_goal1, modified_goal2 = self._spread_goals_apart(
                    goal1, goal2, self.absolute_min_distance
                )
            else:
                # Durante setup, pass-through dei goal formazione
                modified_goal1, modified_goal2 = goal1, goal2
                if self.enable_debug_logs:
                    self.get_logger().info(
                        f"🔧 SETUP MODE: Forming... (distance: {goal_distance:.2f}m)",
                        throttle_duration_sec=2.0
                    )
            
            # Pubblica e esci (non applicare altri vincoli durante setup)
            self.goal_publishers[ns1].publish(modified_goal1)
            self.goal_publishers[ns2].publish(modified_goal2)
            return
        
        # === MODE-SPECIFIC GOAL MODIFICATION (solo dopo formazione raggiunta) ===
        
        if self.current_mode == OperationMode.NORMAL:
            # Pass-through with safety limits only
            if goal_distance > self.absolute_max_distance:
                self.get_logger().warn(
                    f"⚠️ SAFETY: Goals too far ({goal_distance:.2f}m > {self.absolute_max_distance:.2f}m)",
                    throttle_duration_sec=1.0
                )
                modified_goal1, modified_goal2 = self._spread_goals_apart(
                    goal1, goal2, self.absolute_max_distance
                )
            elif goal_distance < self.absolute_min_distance:
                self.get_logger().warn(
                    f"⚠️ SAFETY: Goals too close ({goal_distance:.2f}m < {self.absolute_min_distance:.2f}m)",
                    throttle_duration_sec=1.0
                )
                modified_goal1, modified_goal2 = self._spread_goals_apart(
                    goal1, goal2, self.absolute_min_distance
                )
            else:
                # All OK - pass through
                modified_goal1, modified_goal2 = goal1, goal2
        
        else:
            # APPROACHING, LIFTING, or CLEARING: apply target distance
            modified_goal1, modified_goal2 = self._spread_goals_apart(
                goal1, goal2, target_dist
            )
            
            if self.enable_debug_logs:
                progress = self._get_transition_progress()
                calc_height = self._calculate_object_height(target_dist)
                self.get_logger().info(
                    f"{self._get_mode_emoji()} {self.current_mode.name}: "
                    f"dist {goal_distance:.2f}m → {target_dist:.2f}m "
                    f"(h={calc_height:.3f}m, progress={progress*100:.0f}%)",
                    throttle_duration_sec=0.5
                )
        
        # Publish modified goals
        self.goal_publishers[ns1].publish(modified_goal1)
        self.goal_publishers[ns2].publish(modified_goal2)
    

    def _lifting_distance_height_feedback(self, real_distance: float) -> float:
        """
        Closed-loop correction in LIFTING using measured object height.
        This reduces steady-state errors due to tracking and dynamics.

        Returns:
            A distance command clamped to [absolute_min_distance, absolute_max_distance].
        """
        if not self.enable_height_feedback:
            return self.lifting_h_dist

        current_z = self._calculate_object_height(real_distance)
        err = self.target_object_height - current_z

        # Convert height error into a height command correction, then map back to distance.
        delta_z = self.height_feedback_kp * err
        delta_z = max(-self.height_feedback_max_delta_z, min(self.height_feedback_max_delta_z, delta_z))

        z_cmd = self.target_object_height + delta_z
        z_cmd = max(0.0, min(self.pole_height * 0.95, z_cmd))

        d_cmd = self._calculate_horizontal_distance(z_cmd)
        if d_cmd <= 0.0:
            d_cmd = self.lifting_h_dist

        d_cmd = max(self.absolute_min_distance, min(self.absolute_max_distance, d_cmd))
        return d_cmd
    def _compute_target_distance(self, current_goal_distance: float) -> float:
        """
        Compute target distance based on current mode and transition progress.

        Smooth interpolation during transitions:
        - APPROACHING: current → lifting_dist
        - LIFTING: at lifting_dist (optionally with height feedback)
        - CLEARING: lifting_dist → current
        """
        time_progress = self._get_transition_progress()

        if self.current_mode == OperationMode.NORMAL:
            return current_goal_distance

        elif self.current_mode == OperationMode.APPROACHING:
            # Optional: react to obstacle distance so the ramp does not lag behind fast-approaching obstacles.
            eff_progress = time_progress
            if self.use_distance_based_approach_progress:
                obs = self.obstacle_state.closest_distance
                if obs != float("inf"):
                    denom = max(1e-6, (self.obstacle_approach_distance - self.obstacle_activate_distance))
                    dist_progress = (self.obstacle_approach_distance - obs) / denom
                    dist_progress = max(0.0, min(1.0, dist_progress))
                    eff_progress = max(eff_progress, dist_progress)

            return current_goal_distance + eff_progress * (self.lifting_h_dist - current_goal_distance)

        elif self.current_mode == OperationMode.LIFTING:
            if self._last_real_distance is not None:
                return self._lifting_distance_height_feedback(self._last_real_distance)
            return self.lifting_h_dist

        elif self.current_mode == OperationMode.CLEARING:
            # Smooth ramp from lifting distance back to current
            return self.lifting_h_dist + time_progress * (current_goal_distance - self.lifting_h_dist)

        return current_goal_distance

    def _get_mode_emoji(self) -> str:
        """Get emoji for current mode (for prettier logs)"""
        emojis = {
            OperationMode.NORMAL: "✅",
            OperationMode.APPROACHING: "🔶",
            OperationMode.LIFTING: "🔺",
            OperationMode.CLEARING: "🔽"
        }
        return emojis.get(self.current_mode, "❓")


def main():
    rclpy.init()
    node = TetherConstraintManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
