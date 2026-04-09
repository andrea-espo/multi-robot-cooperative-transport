#!/usr/bin/env python3
import math
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import TransformStamped, TwistStamped


def yaw_from_quaternion(q) -> float:
    """Compute yaw angle (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ImprovedFormationController(Node):
    """
    Leader–follower formation controller with:
    - state machine (NORMAL, SLOWDOWN, SAFETY, REACHED)
    - repositioning waypoint when follower starts ahead of the leader
    - avoidance radius around the leader (no-go disc) in traiettoria
    
    IMPROVED: Precisione migliorata per mantenere distanza target
    """

    def __init__(self):
        super().__init__("improved_formation_controller")

        # Namespaces and frames
        self.declare_parameter("leader_ns", "tb3_3")
        self.declare_parameter("follower_ns", "tb3_4")
        self.declare_parameter("distance", 0.5)     # distanza desiderata dietro il leader
        self.declare_parameter("map_frame", "map")

        # Control gains - MIGLIORATO: leggermente aumentati per migliore tracking
        self.declare_parameter("k_linear", 0.45)      # era 0.4
        self.declare_parameter("k_angular", 2.0)      # era 1.8

        # Safety parameters with hysteresis
        # MIGLIORATO: Zone di sicurezza più conservative per evitare collisioni
        self.declare_parameter("safety_distance_enter", 0.25)    # era 0.30
        self.declare_parameter("safety_distance_exit", 0.35)     # era 0.40
        self.declare_parameter("slow_down_distance_enter", 0.45) # era 0.55
        self.declare_parameter("slow_down_distance_exit", 0.55)  # era 0.65

        # Avoidance radius (zona di safety attorno al leader in traiettoria)
        self.declare_parameter("avoid_radius", 0.40)  # invariato

        # Safe waypoint offsets (waypoint ravvicinato al leader)
        self.declare_parameter("safe_back_offset", 0.10)
        self.declare_parameter("safe_side_offset", 0.10)

        # Limits
        self.declare_parameter("v_max", 0.15)
        self.declare_parameter("w_max", 0.8)
        self.declare_parameter("v_min_safety", 0.0)

        # Tolerances - MIGLIORATO: più strette per maggiore precisione
        self.declare_parameter("pos_tolerance", 0.08)   # era 0.12
        self.declare_parameter("yaw_tolerance", 0.15)   # era 0.20

        # Smoothing - MIGLIORATO: meno smoothing per risposta più rapida
        self.declare_parameter("smoothing_alpha", 0.6)  # era 0.5

        # Angle threshold for "front cone"
        self.declare_parameter("front_cone_angle", 1.22)  # ~70°

        # --- Read parameters ---
        self.leader_ns = self.get_parameter("leader_ns").value
        self.follower_ns = self.get_parameter("follower_ns").value
        self.distance = self.get_parameter("distance").value
        self.map_frame = self.get_parameter("map_frame").value

        self.k_linear = self.get_parameter("k_linear").value
        self.k_angular = self.get_parameter("k_angular").value

        self.safety_dist_enter = self.get_parameter("safety_distance_enter").value
        self.safety_dist_exit = self.get_parameter("safety_distance_exit").value
        self.slowdown_dist_enter = self.get_parameter("slow_down_distance_enter").value
        self.slowdown_dist_exit = self.get_parameter("slow_down_distance_exit").value

        self.avoid_radius = self.get_parameter("avoid_radius").value
        self.safe_back_offset = self.get_parameter("safe_back_offset").value
        self.safe_side_offset = self.get_parameter("safe_side_offset").value

        self.v_max = self.get_parameter("v_max").value
        self.w_max = self.get_parameter("w_max").value
        self.v_min_safety = self.get_parameter("v_min_safety").value

        self.pos_tolerance = self.get_parameter("pos_tolerance").value
        self.yaw_tolerance = self.get_parameter("yaw_tolerance").value

        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.front_cone_angle = self.get_parameter("front_cone_angle").value

        # Consistenza: ensure avoid_radius > safety_dist_enter
        if self.avoid_radius <= self.safety_dist_enter + 0.02:
            self.avoid_radius = self.safety_dist_enter + 0.05
            self.get_logger().warn(
                f"avoid_radius too small, adjusted to {self.avoid_radius:.2f} m"
            )

        # Frames and topics
        self.leader_frame = f"{self.leader_ns}/base_footprint"
        self.follower_frame = f"{self.follower_ns}/base_footprint"
        self.follower_cmd_vel_topic = f"/{self.follower_ns}/cmd_vel"

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.follower_cmd_vel_topic,
            10
        )

        # Safety state machine
        self.current_mode = "NORMAL"  # NORMAL, SLOWDOWN, SAFETY, REACHED

        # Smoothing state
        self.prev_v = 0.0
        self.prev_w = 0.0

        # Repositioning state (per follower davanti al leader)
        self.reposition_active = False
        self.x_safe = 0.0
        self.y_safe = 0.0

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)

        # CSV logging setup
        log_dir = os.path.expanduser("~/formation_logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = self.get_clock().now().nanoseconds
        log_filename = f"{log_dir}/formation_log_{timestamp}.csv"
        
        self.log_file = open(log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        # Write CSV header
        self.csv_writer.writerow([
            't', 'mode', 'x_F', 'y_F', 'yaw_F_deg', 'x_L', 'y_L', 'yaw_L_deg',
            'x_tgt', 'y_tgt', 'd_tgt', 'd_ldr', 'leader_front',
            'head_err_deg', 'yaw_err_deg', 'v_raw', 'w_raw',
            'v_unclamped', 'w_unclamped', 'v_cmd', 'w_cmd',
            'v_filt', 'w_filt', 'v_sat', 'w_sat'
        ])
        self.log_file.flush()

        self.get_logger().info(f"CSV logging to: {log_filename}")

        self.get_logger().info(
            f"Improved formation controller started (PRECISION TUNED):\n"
            f"  Leader: {self.leader_ns}, Follower: {self.follower_ns}\n"
            f"  Formation distance: {self.distance:.2f} m\n"
            f"  Safety zone: {self.safety_dist_enter:.2f}–{self.safety_dist_exit:.2f} m\n"
            f"  Slowdown zone: {self.slowdown_dist_enter:.2f}–{self.slowdown_dist_exit:.2f} m\n"
            f"  Avoid radius: {self.avoid_radius:.2f} m\n"
            f"  Safe waypoint offsets: back={self.safe_back_offset:.2f} m, "
            f"side={self.safe_side_offset:.2f} m"
        )

    def get_pose_in_map(self, frame_id: str):
        """Get pose (x, y, yaw) of frame_id in map frame."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f"TF lookup failed for '{frame_id}': {ex}",
                throttle_duration_sec=2.0
            )
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return x, y, yaw

    def is_leader_in_front(self, leader_x_follower: float, leader_y_follower: float) -> bool:
        """Check if leader is in front cone of follower."""
        if leader_x_follower <= 0:
            return False
        angle_to_leader = abs(math.atan2(leader_y_follower, leader_x_follower))
        return angle_to_leader < self.front_cone_angle

    def update_mode(self, distance_to_leader: float) -> str:
        """Update control mode with hysteresis."""

        if self.current_mode == "SAFETY":
            if distance_to_leader > self.safety_dist_exit:
                return "SLOWDOWN"
            return "SAFETY"

        if self.current_mode == "SLOWDOWN":
            if distance_to_leader < self.safety_dist_enter:
                return "SAFETY"
            if distance_to_leader > self.slowdown_dist_exit:
                return "NORMAL"
            return "SLOWDOWN"

        # NORMAL o REACHED
        if distance_to_leader < self.slowdown_dist_enter:
            if distance_to_leader < self.safety_dist_enter:
                return "SAFETY"
            return "SLOWDOWN"

        return "NORMAL"

    def control_loop(self):
        """Main control loop with repositioning and avoidance radius."""

        # --- Leader pose ---
        leader_pose = self.get_pose_in_map(self.leader_frame)
        if leader_pose is None:
            return
        x_L, y_L, yaw_L = leader_pose

        # --- Follower pose ---
        follower_pose = self.get_pose_in_map(self.follower_frame)
        if follower_pose is None:
            return
        x_F, y_F, yaw_F = follower_pose

        # ============================================================
        # GEOMETRIA RELATIVA
        # ============================================================
        dx_to_leader = x_L - x_F
        dy_to_leader = y_L - y_F
        distance_to_leader = math.hypot(dx_to_leader, dy_to_leader)

        # Follower nel frame del leader
        cos_L = math.cos(yaw_L)
        sin_L = math.sin(yaw_L)
        dx_FL = x_F - x_L
        dy_FL = y_F - y_L
        dx_LF = cos_L * dx_FL + sin_L * dy_FL
        dy_LF = -sin_L * dx_FL + cos_L * dy_FL

        ahead_of_leader = (
            dx_LF > 0.0
            and abs(dy_LF) < 0.4
            and distance_to_leader < 1.0
        )

        # ============================================================
        # REPOSITIONING: WAYPOINT SAFE RAVVICINATO
        # ============================================================
        if not self.reposition_active and ahead_of_leader:
            # lato scelto in base al segno di dy_LF
            side = 1.0 if dy_LF >= 0.0 else -1.0

            d_back = self.distance + self.safe_back_offset
            d_side = self.distance + self.safe_side_offset

            # Waypoint dietro e di lato al leader
            self.x_safe = x_L - d_back * cos_L - side * d_side * sin_L
            self.y_safe = y_L - d_back * sin_L + side * d_side * cos_L
            self.reposition_active = True

            self.get_logger().info(
                f"Entering REPOSITION: d_back={d_back:.2f}, d_side={d_side:.2f}"
            )

        # ============================================================
        # SELEZIONE DEL GOAL
        # ============================================================
        if self.reposition_active:
            x_goal = self.x_safe
            y_goal = self.y_safe
            yaw_target = yaw_L

            # quando il waypoint safe è raggiunto, torna al target di formazione
            if math.hypot(x_goal - x_F, y_goal - y_F) < self.pos_tolerance:
                self.reposition_active = False
                self.get_logger().info(
                    "Safe waypoint reached, switching back to formation target"
                )
                x_goal = x_L - self.distance * cos_L
                y_goal = y_L - self.distance * sin_L
                yaw_target = yaw_L
        else:
            # target di formazione standard dietro il leader
            x_goal = x_L - self.distance * cos_L
            y_goal = y_L - self.distance * sin_L
            yaw_target = yaw_L

        # Errore di posizione nel frame mappa
        dx = x_goal - x_F
        dy = y_goal - y_F
        distance_to_target = math.hypot(dx, dy)

        # Errore di yaw rispetto all'orientazione desiderata
        yaw_error = normalize_angle(yaw_target - yaw_F)

        # Target raggiunto (solo se non in reposition)
        if (
            not self.reposition_active
            and distance_to_target < self.pos_tolerance
            and abs(yaw_error) < self.yaw_tolerance
        ):
            if self.current_mode != "REACHED":
                self.current_mode = "REACHED"
                self.get_logger().info("TARGET REACHED - Formation maintained")
            self.publish_zero_velocity()
            return

        # ============================================================
        # ESPRIMI ERRORI NEL FRAME DEL FOLLOWER
        # ============================================================
        cos_F = math.cos(yaw_F)
        sin_F = math.sin(yaw_F)

        # Target in frame follower
        error_x_follower = cos_F * dx + sin_F * dy
        error_y_follower = -sin_F * dx + cos_F * dy

        # Leader in frame follower
        leader_x_follower = cos_F * dx_to_leader + sin_F * dy_to_leader
        leader_y_follower = -sin_F * dx_to_leader + cos_F * dy_to_leader

        # Direzione pura verso il goal (attrazione)
        angle_to_target = math.atan2(dy, dx)

        # Direzione di repulsione (via dal leader)
        angle_repulsion = math.atan2(-dy_to_leader, -dx_to_leader)

        # ============================================================
        # LOGICA DI EVITAMENTO (NO-GO DISC ATTORNO AL LEADER)
        # ============================================================
        # angle_field combina attrazione al goal e repulsione dal leader
        angle_field = angle_to_target
        if distance_to_leader < self.avoid_radius and distance_to_leader > 1e-3:
            # λ = 0 vicino a safety_dist_enter (repulsione forte),
            #     1 vicino a avoid_radius (quasi solo attrazione)
            denom = max(self.avoid_radius - self.safety_dist_enter, 1e-3)
            lam = (distance_to_leader - self.safety_dist_enter) / denom
            lam = max(0.0, min(1.0, lam))

            x_field = lam * math.cos(angle_to_target) + (1.0 - lam) * math.cos(angle_repulsion)
            y_field = lam * math.sin(angle_to_target) + (1.0 - lam) * math.sin(angle_repulsion)
            angle_field = math.atan2(y_field, x_field)

        heading_error = normalize_angle(angle_field - yaw_F)

        # leader nel cono frontale del follower (per logica di safety)
        leader_in_front = self.is_leader_in_front(leader_x_follower, leader_y_follower)

        # Aggiornamento modo di sicurezza
        self.current_mode = self.update_mode(distance_to_leader)

        # ============================================================
        # CONTROLLO
        # MIGLIORATO: velocità leggermente più aggressive per precisione
        # ============================================================
        v = 0.0
        w = 0.0

        if self.current_mode == "SAFETY":
            # Molto vicino al leader
            if leader_in_front:
                v = self.v_min_safety
                if distance_to_leader < (self.safety_dist_enter * 0.9):
                    # forte rotazione laterale se estremamente vicino
                    if leader_y_follower > 0:
                        w = -0.9  # MIGLIORATO: era -0.8
                    else:
                        w = 0.9   # MIGLIORATO: era 0.8
                else:
                    w = self.k_angular * heading_error * 0.5
            else:
                # leader non direttamente davanti: avvicinamento cauto
                v = self.k_linear * error_x_follower * 0.35  # MIGLIORATO: era 0.3
                w = self.k_angular * heading_error * 0.7

        elif self.current_mode == "SLOWDOWN":
            # Regione di rallentamento
            if leader_in_front:
                range_size = self.slowdown_dist_exit - self.safety_dist_exit
                range_size = max(range_size, 1e-3)
                speed_factor = (distance_to_leader - self.safety_dist_exit) / range_size
                speed_factor = max(0.25, min(1.0, speed_factor))  # MIGLIORATO: min era 0.2

                v = self.k_linear * error_x_follower * speed_factor

                if distance_to_target > 0.20:  
                    w = self.k_angular * heading_error * 0.85  # MIGLIORATO: era 0.8
                else:
                    w = self.k_angular * yaw_error * 0.85
            else:
                v = self.k_linear * error_x_follower * 0.90  

                if distance_to_target > 0.20:  
                    w = self.k_angular * heading_error
                else:
                    w = self.k_angular * yaw_error

        else:  # NORMAL
            # Controllo standard
            v = self.k_linear * error_x_follower

            if distance_to_target > 0.2:  
                w = self.k_angular * heading_error
            else:
                w = self.k_angular * yaw_error

        # Raw controller outputs (before safety scaling/limits), useful for logging/plots
        v_raw = v
        w_raw = w

        # MIGLIORATO: limitazione velocità dentro avoid_radius più graduale
        if distance_to_leader < self.avoid_radius:
            # Scala da 0.5 (vicino) a 1.0 (lontano) invece di fisso 0.4
            speed_scale = 0.5 + 0.5 * (distance_to_leader / self.avoid_radius)
            v = min(v, self.v_max * speed_scale)

        # Values after avoidance scaling but before hard saturation
        v_unclamped = v
        w_unclamped = w

        # Limiti su v e w
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))
        # Saturation flags (after avoidance scaling but before filtering)
        v_sat = (v != v_unclamped)
        w_sat = (w != w_unclamped)

        # Store commanded (pre-filter) values for logging
        v_cmd = v
        w_cmd = w


        # Smoothing (meno aggressivo per risposta più rapida)
        alpha = self.smoothing_alpha if self.current_mode != "SAFETY" else 0.7  # era 0.8
        v_filtered = alpha * v + (1.0 - alpha) * self.prev_v
        w_filtered = alpha * w + (1.0 - alpha) * self.prev_w

        self.prev_v = v_filtered
        self.prev_w = w_filtered

        # Pubblica comando
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.follower_frame
        cmd.twist.linear.x = v_filtered
        cmd.twist.angular.z = w_filtered
        self.cmd_pub.publish(cmd)

        # Timestamp for plotting (ROS time in seconds)
        t_sec = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f"[{self.current_mode:8s}] d_tgt={distance_to_target:.3f}m "
            f"d_ldr={distance_to_leader:.3f}m "
            f"{'[FRONT]' if leader_in_front else '[SIDE]'} | "
            f"head={math.degrees(heading_error):5.1f}° "
            f"yaw={math.degrees(yaw_error):5.1f}° | "
            f"v={v_filtered:+.3f} w={w_filtered:+.3f}",
            throttle_duration_sec=0.5
        )
        # Machine-friendly log line (single line, key=value pairs) for copy/paste plotting
        self.get_logger().info(
            "PLOT "
            f"t={t_sec:.3f} "
            f"mode={self.current_mode} "
            f"d_tgt={distance_to_target:.6f} "
            f"d_ldr={distance_to_leader:.6f} "
            f"leader_front={int(leader_in_front)} "
            f"head_err_deg={math.degrees(heading_error):.6f} "
            f"yaw_err_deg={math.degrees(yaw_error):.6f} "
            f"v_raw={v_raw:.6f} w_raw={w_raw:.6f} "
            f"v_unclamped={v_unclamped:.6f} w_unclamped={w_unclamped:.6f} "
            f"v_cmd={v_cmd:.6f} w_cmd={w_cmd:.6f} "
            f"v_filt={v_filtered:.6f} w_filt={w_filtered:.6f} "
            f"v_sat={int(v_sat)} w_sat={int(w_sat)}",
            throttle_duration_sec=0.5
        )
        
        # Write data to CSV
        self.csv_writer.writerow([
            t_sec, self.current_mode, 
            x_F, y_F, math.degrees(yaw_F),
            x_L, y_L, math.degrees(yaw_L),
            x_goal, y_goal,
            distance_to_target, distance_to_leader, int(leader_in_front),
            math.degrees(heading_error), math.degrees(yaw_error),
            v_raw, w_raw, v_unclamped, w_unclamped,
            v_cmd, w_cmd, v_filtered, w_filtered,
            int(v_sat), int(w_sat)
        ])
        self.log_file.flush()  # Ensure data is written immediately


    def publish_zero_velocity(self):
        """Stop the robot and reset smoothing state."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.follower_frame
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        self.prev_v = 0.0
        self.prev_w = 0.0

    def destroy_node(self):
        """Close CSV file before destroying node."""
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
            self.get_logger().info("CSV log file closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedFormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
