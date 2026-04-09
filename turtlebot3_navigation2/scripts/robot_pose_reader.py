#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time


def yaw_from_quaternion(q):
    """Compute yaw angle (rotation around Z) from a geometry_msgs/Quaternion."""
    # Standard formula for ZYX (roll-pitch-yaw) extracting yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotPoseReader(Node):
    def __init__(self):
        super().__init__('robot_pose_reader')

        # TF2 buffer and listener to read transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List of robot base frames (adjust names to your robots)
        self.robot_frames = [
            'tb3_3/base_footprint',
            'tb3_4/base_footprint',
        ]

        # Periodic timer to read poses
        self.timer = self.create_timer(1.0, self.timer_callback)  # 10 Hz

    def get_pose_in_map(self, robot_frame):
        """Return (x, y, yaw) of robot_frame expressed in map frame."""
        try:
            # Time() without arguments means: latest available transform
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',          # target frame
                robot_frame,    # source frame
                Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f'Cannot transform {robot_frame} to map: {ex}')
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = yaw_from_quaternion(q)

        return x, y, yaw

    def timer_callback(self):
        for frame in self.robot_frames:
            pose = self.get_pose_in_map(frame)
            if pose is not None:
                x, y, yaw = pose
                self.get_logger().info(
                    f'{frame} in map: x={x:.2f} m, y={y:.2f} m, yaw={yaw:.2f} rad'
                )


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

