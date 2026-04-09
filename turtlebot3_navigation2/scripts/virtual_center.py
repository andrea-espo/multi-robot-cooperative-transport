#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped, PoseStamped


class FormationCenter(Node):
    """
    Virtual formation center (virtual leader).
    It integrates a unicycle model from a commanded TwistStamped on cmd_topic
    and publishes the pose as PoseStamped on pose_topic.

    This version adds:
      - Low-pass filtering on v and w (to smooth teleop steps)
      - Saturation limits on v and w
      - Command timeout: if cmd is stale, it decays to zero
    """

    def __init__(self):
        super().__init__("formation_center")

        # Parameters: initial pose of the formation center
        self.declare_parameter("x0", -0.13)
        self.declare_parameter("y0", 1.77)
        self.declare_parameter("theta0", 3.14)

        # Topics
        self.declare_parameter("cmd_topic", "/formation/cmd_vel")
        self.declare_parameter("pose_topic", "/formation/pose")

        # Integration rate
        self.declare_parameter("update_rate", 50.0)  # Hz

        # Frame id for PoseStamped
        self.declare_parameter("frame_id", "map")

        # Smoothing (low-pass) parameters
        # alpha in [0,1]: smaller -> smoother (more inertia), larger -> more reactive
        self.declare_parameter("alpha_v", 0.6)
        self.declare_parameter("alpha_w", 0.5)

        # Saturation limits for the virtual leader command
        self.declare_parameter("v_max", 0.10)   # [m/s]
        self.declare_parameter("w_max", 0.60)   # [rad/s]

        # Command timeout (if cmd is older than this, treat cmd as zero)
        self.declare_parameter("cmd_timeout", 0.7)  # [s]

        # Internal state: pose of the virtual formation center
        self.x = float(self.get_parameter("x0").value)
        self.y = float(self.get_parameter("y0").value)
        self.theta = float(self.get_parameter("theta0").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.alpha_v = float(self.get_parameter("alpha_v").value)
        self.alpha_w = float(self.get_parameter("alpha_w").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout").value)

        # Last commanded TwistStamped (raw)
        self.current_cmd = TwistStamped()
        self.current_cmd.twist.linear.x = 0.0
        self.current_cmd.twist.angular.z = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Filtered command state
        self.v_f = 0.0
        self.w_f = 0.0

        # Subscriber for teleop / controller command of the formation frame
        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # Publisher for the pose of the formation center
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10
        )

        # Timer for integration (fixed dt)
        self.dt = 1.0 / max(self.update_rate, 1e-6)
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            "FormationCenter started: "
            f"x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f} rad, "
            f"cmd_topic={self.cmd_topic}, pose_topic={self.pose_topic}, frame_id={self.frame_id}, "
            f"update_rate={self.update_rate:.1f} Hz, "
            f"alpha_v={self.alpha_v:.2f}, alpha_w={self.alpha_w:.2f}, "
            f"v_max={self.v_max:.2f}, w_max={self.w_max:.2f}, "
            f"cmd_timeout={self.cmd_timeout_s:.2f}s"
        )

    def cmd_callback(self, msg: TwistStamped):
        """Store the last commanded TwistStamped for the virtual formation center."""
        self.current_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """Integrate the unicycle model and publish the formation center pose."""
        now = self.get_clock().now()

        # If command is stale, treat target cmd as zero
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout_s:
            v_target = 0.0
            w_target = 0.0
        else:
            v_target = float(self.current_cmd.twist.linear.x)
            w_target = float(self.current_cmd.twist.angular.z)

        # Saturate target commands
        v_target = max(-self.v_max, min(self.v_max, v_target))
        w_target = max(-self.w_max, min(self.w_max, w_target))

        # Low-pass filter on commands
        a_v = max(0.0, min(1.0, self.alpha_v))
        a_w = max(0.0, min(1.0, self.alpha_w))
        self.v_f = a_v * v_target + (1.0 - a_v) * self.v_f
        self.w_f = a_w * w_target + (1.0 - a_w) * self.w_f

        # Integrate pose with fixed time step dt
        self.x += self.v_f * math.cos(self.theta) * self.dt
        self.y += self.v_f * math.sin(self.theta) * self.dt
        self.theta = self.normalize_angle(self.theta + self.w_f * self.dt)

        # Publish pose as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = self.frame_id

        pose_msg.pose.position.x = float(self.x)
        pose_msg.pose.position.y = float(self.y)
        pose_msg.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion (rotation around Z only)
        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """Convert yaw angle (around Z) to a unit quaternion."""
        half = yaw * 0.5
        cz = math.cos(half)
        sz = math.sin(half)
        # Rotation only around Z axis: q = (0, 0, sin(yaw/2), cos(yaw/2))
        return 0.0, 0.0, sz, cz


def main(args=None):
    rclpy.init(args=args)
    node = FormationCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

