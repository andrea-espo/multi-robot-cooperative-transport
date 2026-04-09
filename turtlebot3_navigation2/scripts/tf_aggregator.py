#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class TfAggregator(Node):
    def __init__(self):
        super().__init__('tf_aggregator')

        # Parameter: list of robot namespaces
        robot_names_param = self.declare_parameter(
            'robot_names',
            ['tb3_3', 'tb3_4']  # change this list according to your robots
        )
        self.robot_names = list(robot_names_param.get_parameter_value().string_array_value)

        # QoS profile for /tf (dynamic transforms)
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS profile for /tf_static (static transforms)
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Global publishers
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)

        # Keep references to subscriptions to avoid garbage collection
        self.tf_subs = []

        for name in self.robot_names:
            tf_topic = f'/{name}/tf'
            tf_static_topic = f'/{name}/tf_static'

            self.get_logger().info(f"Subscribing to {tf_topic} and {tf_static_topic}")

            self.tf_subs.append(
                self.create_subscription(
                    TFMessage, tf_topic, self.tf_callback, tf_qos
                )
            )
            self.tf_subs.append(
                self.create_subscription(
                    TFMessage, tf_static_topic, self.tf_static_callback, tf_static_qos
                )
            )

    def tf_callback(self, msg: TFMessage):
        """Republish dynamic TF messages to global /tf."""
        self.tf_pub.publish(msg)

    def tf_static_callback(self, msg: TFMessage):
        """Republish static TF messages to global /tf_static."""
        self.tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

