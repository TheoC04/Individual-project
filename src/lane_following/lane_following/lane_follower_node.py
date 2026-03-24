import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import numpy as np
import cv2


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # Subscribe to compressed camera images
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # Publish velocity commands
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Processed image publisher
        self.debug_pub = self.create_publisher(
            CompressedImage,
            '/line_follower/image/compressed', #needs /compressed at the end so it uses that plugin to publish
            10
        )

        self.get_logger().info("Line follower node started")

    def image_callback(self, msg):

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # Resize (faster processing)
        frame = cv2.resize(frame, (320, 240))

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define yellow range in HSV
        lower_yellow = np.array([20, 100, 100])   # H:20, S:100, V:100
        upper_yellow = np.array([30, 255, 255])   # H:30, S:255, V:255

        # Create a mask for yellow color
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Region of interest (bottom part)
        h, w = mask.shape
        roi = mask[int(h * 0.4):h, :]

        # Create empty mask image
        clean = np.zeros_like(mask)
        clean[int(h*0.4):h, :] = roi

        # Find centroid
        M = cv2.moments(roi)

        twist = Twist()

        if M["m00"] > 0:

            cx = int(M["m10"] / M["m00"])

            error = cx - (w // 2)

            # Proportional steering
            Kp = 0.005
            twist.linear.x = 0.2
            twist.angular.z = -Kp * error

        else:
            # No line detected → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

        # Encode cleaned image
        success, encoded = cv2.imencode('.jpg', clean)

        if success:
            self.get_logger().info(f"Publishing processed image")
            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = encoded.tobytes()

            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()