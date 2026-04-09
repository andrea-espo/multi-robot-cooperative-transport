#!/usr/bin/env python3
import math
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32

from nav2_msgs.action import NavigateToPose


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


def quaternion_from_yaw(yaw: float):
    """Return (z,w) quaternion components for pure yaw rotation."""
    half = 0.5 * yaw
    return math.sin(half), math.cos(half)


class FormationFollower(Node):
    """
    Dynamic-following approach:
      - Compute the desired goal pose at distance d from /formation/pose
      - Publish it continuously to /<robot_ns>/goal_update (GoalUpdater BT node subscribes)
      - Send ONE NavigateToPose action goal with a custom BT (follow point)
        so Nav2 handles obstacle avoidance.

    Improvements in this version:
      - Goal update rate increased (default 20 Hz)
      - Goal updates are filtered + rate-limited + thresholded to reduce "micro-preemption"
        and reduce stop-and-go when approaching obstacles.
    """

    def __init__(self):
        super().__init__("formation_follower_follow_point")

        # Parameters: robot and frames
        self.declare_parameter("robot_ns", "tb3_3")
        self.declare_parameter("map_frame", "map")

        # Lateral distance parameters
        self.declare_parameter("distance_nominal", 0.2)   # d_nom  [m]
        self.declare_parameter("distance_open", 1.0)      # d_open [m]
        self.declare_parameter("side_sign", -1.0)         # +1 left, -1 right

        # Lambda deformation control
        self.declare_parameter("use_lambda", True)
        self.declare_parameter("lambda_init", 0.0)
        self.declare_parameter("lambda_topic", "/formation/lambda")

        # Publish rate for goal updates (PoseStamped)
        self.declare_parameter("goal_update_hz", 30.0)

        # Goal update policy (to make motion smoother)
        # - min_period: do not publish faster than this (even if goal_update_hz is higher)
        # - min_translation/min_yaw: publish only if goal moved enough
        # - smoothing_alpha: low-pass filter on goal pose (0 -> no updates, 1 -> no filter)
        self.declare_parameter("goal_update_min_period", 0.05)      # [s] 10 Hz max publish by default
        self.declare_parameter("goal_update_min_translation", 0.01) # [m]
        self.declare_parameter("goal_update_min_yaw", 0.02)         # [rad] ~3 deg
        self.declare_parameter("goal_smoothing_alpha", 0.6)        # [0..1]

        # Behavior tree XML path (default: same folder as this script)
        self.declare_parameter("bt_xml_path", "")

        # Read parameters
        self.robot_ns = str(self.get_parameter("robot_ns").value)
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.d_nominal = float(self.get_parameter("distance_nominal").value)
        self.d_open = float(self.get_parameter("distance_open").value)
        self.side_sign = float(self.get_parameter("side_sign").value)

        self.use_lambda = bool(self.get_parameter("use_lambda").value)
        self.lambda_val = float(self.get_parameter("lambda_init").value)
        self.lambda_topic = str(self.get_parameter("lambda_topic").value)

        self.goal_update_hz = float(self.get_parameter("goal_update_hz").value)
        self.goal_update_min_period = float(self.get_parameter("goal_update_min_period").value)
        self.goal_update_min_translation = float(self.get_parameter("goal_update_min_translation").value)
        self.goal_update_min_yaw = float(self.get_parameter("goal_update_min_yaw").value)
        self.goal_smoothing_alpha = float(self.get_parameter("goal_smoothing_alpha").value)

        bt_xml_path = str(self.get_parameter("bt_xml_path").value).strip()
        if bt_xml_path:
            self.bt_xml_path = bt_xml_path
        else:
            self.bt_xml_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "follow_point_tb3_3.xml",
            )

        # Topics/frames
        self.follower_frame = f"{self.robot_ns}/base_footprint"
        self.formation_pose_topic = "/formation/pose"

        # This is the topic used by GoalUpdater; default is "goal_update" in Nav2
        # Under namespace tb3_3 it becomes /tb3_3/goal_update
        self.goal_update_topic = f"/{self.robot_ns}/formation_goal"

        # Optional debug topic for RViz
        self.debug_goal_topic = f"/{self.robot_ns}/formation_goal_debug"

        # Nav2 action
        self.nav2_action_name = f"/{self.robot_ns}/navigate_to_pose"

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Center pose subscriber
        self.center_pose: Optional[PoseStamped] = None
        self.center_sub = self.create_subscription(
            PoseStamped, self.formation_pose_topic, self.center_callback, 10
        )

        # Lambda subscriber (optional)
        if self.use_lambda:
            self.lambda_sub = self.create_subscription(
                Float32, self.lambda_topic, self.lambda_callback, 10
            )
        else:
            self.lambda_sub = None

        # Publishers
        self.goal_update_pub = self.create_publisher(PoseStamped, self.goal_update_topic, 10)
        self.debug_goal_pub = self.create_publisher(PoseStamped, self.debug_goal_topic, 10)

        # Nav2 Action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        # Action state
        self.goal_handle = None
        self.action_active = False

        # Goal publish state (filter + gating)
        self.filtered_goal_state: Optional[Tuple[float, float, float]] = None  # (x,y,yaw)
        self.last_published_goal: Optional[Tuple[float, float, float]] = None
        self.last_publish_time = self.get_clock().now()

        # Main loop (compute at goal_update_hz, publish may be throttled by min_period)
        period = 1.0 / max(1.0, self.goal_update_hz)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(
            "FormationFollower (Follow Dynamic Point) started:\n"
            f"  robot_ns: {self.robot_ns}\n"
            f"  follower_frame: {self.follower_frame}\n"
            f"  map_frame: {self.map_frame}\n"
            f"  formation_pose_topic: {self.formation_pose_topic}\n"
            f"  goal_update_topic: {self.goal_update_topic}\n"
            f"  debug_goal_topic: {self.debug_goal_topic}\n"
            f"  nav2_action_name: {self.nav2_action_name}\n"
            f"  bt_xml_path: {self.bt_xml_path}\n"
            f"  goal_update_hz: {self.goal_update_hz}\n"
            f"  goal_update_min_period: {self.goal_update_min_period}\n"
            f"  goal_update_min_translation: {self.goal_update_min_translation}\n"
            f"  goal_update_min_yaw: {self.goal_update_min_yaw}\n"
            f"  goal_smoothing_alpha: {self.goal_smoothing_alpha}\n"
        )

    def center_callback(self, msg: PoseStamped):
        self.center_pose = msg

    def lambda_callback(self, msg: Float32):
        val = float(msg.data)
        self.lambda_val = max(0.0, min(1.0, val))

    def get_pose_in_map(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        """Return (x,y,yaw) of a TF frame in map."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, frame_id, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f"TF lookup failed for '{frame_id}': {ex}",
                throttle_duration_sec=1.0,
            )
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return x, y, yaw

    def compute_goal_pose(self) -> Optional[PoseStamped]:
        """Compute PoseStamped goal at lateral offset from the formation center pose."""
        if self.center_pose is None:
            return None

        # Ensure follower pose is available (TF healthy)
        follower_pose = self.get_pose_in_map(self.follower_frame)
        if follower_pose is None:
            return None

        cp = self.center_pose.pose
        x_c = cp.position.x
        y_c = cp.position.y
        yaw_c = yaw_from_quaternion(cp.orientation)

        # Effective distance
        if self.use_lambda:
            lam = max(0.0, min(1.0, self.lambda_val))
            d_eff = (1.0 - lam) * self.d_nominal + lam * self.d_open
        else:
            d_eff = self.d_nominal

        # Lateral offset
        cos_c = math.cos(yaw_c)
        sin_c = math.sin(yaw_c)
        s = self.side_sign

        x_goal = x_c - s * d_eff * sin_c
        y_goal = y_c + s * d_eff * cos_c
        yaw_goal = yaw_c

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = x_goal
        goal_msg.pose.position.y = y_goal
        goal_msg.pose.position.z = 0.0
        qz, qw = quaternion_from_yaw(yaw_goal)
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        return goal_msg

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal was rejected.")
            self.goal_handle = None
            self.action_active = False
            return

        self.get_logger().info("Nav2 goal accepted (Follow Dynamic Point BT running).")
        self.goal_handle = goal_handle
        self.action_active = True

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        self.get_logger().warn(
            f"Nav2 action finished with status={status}. Will re-send when goal is available."
        )
        self.action_active = False
        self.goal_handle = None

    def ensure_action_started(self, initial_goal: PoseStamped):
        """Send one NavigateToPose goal using the follow-point BT."""
        if self.action_active:
            return

        if not os.path.isfile(self.bt_xml_path):
            self.get_logger().error(
                f"BT XML not found: {self.bt_xml_path}. "
                "Create 'follow_point_tb3_3.xml' next to this script or set -p bt_xml_path:=/abs/path.xml"
            )
            return

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Nav2 action server not available: {self.nav2_action_name}",
                throttle_duration_sec=2.0,
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = initial_goal
        goal.behavior_tree = self.bt_xml_path

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _filter_and_gate_goal(self, raw: PoseStamped) -> Optional[PoseStamped]:
        """
        Apply low-pass filtering + thresholding + rate limiting to goal updates.
        Returns a PoseStamped to publish, or None if we should skip publishing this cycle.
        """
        raw_x = float(raw.pose.position.x)
        raw_y = float(raw.pose.position.y)
        raw_yaw = yaw_from_quaternion(raw.pose.orientation)

        # Low-pass filter
        a = max(0.0, min(1.0, self.goal_smoothing_alpha))
        if self.filtered_goal_state is None:
            x_f, y_f, yaw_f = raw_x, raw_y, raw_yaw
        else:
            x_prev, y_prev, yaw_prev = self.filtered_goal_state
            x_f = a * raw_x + (1.0 - a) * x_prev
            y_f = a * raw_y + (1.0 - a) * y_prev
            # Proper yaw filtering on the circle
            dyaw = normalize_angle(raw_yaw - yaw_prev)
            yaw_f = normalize_angle(yaw_prev + a * dyaw)

        self.filtered_goal_state = (x_f, y_f, yaw_f)

        # Rate limiting
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        if dt < max(0.0, self.goal_update_min_period):
            return None

        # Thresholding
        if self.last_published_goal is not None:
            x_last, y_last, yaw_last = self.last_published_goal
            dist = math.hypot(x_f - x_last, y_f - y_last)
            dyaw_abs = abs(normalize_angle(yaw_f - yaw_last))
            if dist < self.goal_update_min_translation and dyaw_abs < self.goal_update_min_yaw:
                return None

        # Build filtered goal message
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x_f
        msg.pose.position.y = y_f
        msg.pose.position.z = 0.0
        qz, qw = quaternion_from_yaw(yaw_f)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.last_publish_time = now
        self.last_published_goal = (x_f, y_f, yaw_f)
        return msg

    def loop(self):
        raw_goal = self.compute_goal_pose()
        if raw_goal is None:
            return

        # Start Nav2 action once; use the raw goal for initial start.
        self.ensure_action_started(raw_goal)

        # Filter + gate the goal updates for smoother motion
        goal_to_publish = self._filter_and_gate_goal(raw_goal)
        if goal_to_publish is None:
            return

        self.goal_update_pub.publish(goal_to_publish)
        self.debug_goal_pub.publish(goal_to_publish)


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
