#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Subscribe to target point from vision node
        self.subscription = self.create_subscription(
            Point,
            '/target_point',
            self.point_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Image width (must match camera processing)
        self.image_width = 320

        # Parameters
        self.k_steer = 2.0
        self.linear_speed = 0.2

    def point_callback(self, msg):
        twist = Twist()

        # Extract target point
        x_target = msg.x
        y_target = msg.y

        # Compute error from center
        center = self.image_width / 2.0
        dx = x_target - center

        # Lookahead distance (vertical distance in image)
        dy = 240 - y_target  # assuming 240 height

        # Compute steering angle
        angle = math.atan2(dx, dy)

        # Apply control
        twist.linear.x = self.linear_speed
        twist.angular.z = -self.k_steer * angle

        # Clamp steering (important!)
        twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()