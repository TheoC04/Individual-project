#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from messages.msg import Sign, Float32Stamped

confidence_threshold = 0.6  # Only consider signs above this confidence level


class SignControl(Node):

    def __init__(self):
        super().__init__('sign_control_node')

        self.sub = self.create_subscription(
            Sign,
            "/detected_sign",
            self.sign_callback,
            10
        )

        # Publish speed only
        self.pub = self.create_publisher(
            Float32Stamped,
            "/speed_limit",
            10
        )

        self.current_speed = 0.0

        self.get_logger().info("Sign Control Node Started")

    def sign_callback(self, msg):
        sign = msg.label
        self.get_logger().info(f"Received sign: {sign}")
        confidence = msg.confidence
        self.get_logger().info(f"Confidence: {confidence:.2f}")
        self.header = msg.header
        t_capture = Time.from_msg(msg.header.stamp)

        # --- Decision logic ---
        if sign == "stop" and confidence >= confidence_threshold:
            self.current_speed = 0.0

        elif sign == "Speed Limit 10" and confidence >= confidence_threshold:
            self.current_speed = 0.1

        elif sign == "Speed Limit 20" and confidence >= confidence_threshold:
            self.current_speed = 0.2

        elif sign == "Speed Limit 30" and confidence >= confidence_threshold:
            self.current_speed = 0.3

        elif sign == "Speed Limit 50" and confidence >= confidence_threshold:
            self.current_speed = 0.5

        elif sign == "go" and confidence >= confidence_threshold:
            self.current_speed = 0.4

        else:
            self.get_logger().info(f"Unknown sign: {sign}")

        # Publish speed
        out = Float32Stamped()
        out.data = self.current_speed
        out.header.stamp = msg.header.stamp  # Use the same timestamp as the input sign message
        self.pub.publish(out)

        t_now = self.get_clock().now()
        delay = (t_now - t_capture).nanoseconds * 1e-9
        self.get_logger().info(f"Processing time: {delay:.3f} seconds")
        self.get_logger().info(f"Sign: {sign} | Speed set to {self.current_speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SignControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
