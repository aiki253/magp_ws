#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMuxNode(Node):
    def __init__(self):
        super().__init__('joy_mux_node')

        self.declare_parameter('input_topics', ['/joy0', '/joy1'])
        self.declare_parameter('output_topic', '/joy')

        input_topics = self.get_parameter('input_topics').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Joy, output_topic, 10)

        self.subs = []
        for topic in input_topics:
            sub = self.create_subscription(
                Joy,
                topic,
                self.joy_callback,
                10
            )
            self.subs.append(sub)

        self.get_logger().info(
            f'joy_mux: {input_topics} -> {output_topic}'
        )

    def joy_callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
