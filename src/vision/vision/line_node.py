import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetector:
    def __init__(self, frame):
        self.orig = frame
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def preprocess(self):
        blurred = cv2.GaussianBlur(self.gray, (5, 5), 0)
        return cv2.Canny(blurred, 50, 150)

    def detect_lines(self, edges):
        raw = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            50,
            minLineLength=50,
            maxLineGap=10
        )

        lines = []
        if raw is not None:
            for l in raw:
                lines.append(tuple(l[0]))
        return lines

    def draw_lines(self, lines):
        out = self.orig.copy()
        for x1, y1, x2, y2 in lines:
            cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return out


class LineDetectionNode(Node):

    def __init__(self):
        super().__init__('line_detection_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/camera/line_image',
            10
        )

        self.get_logger().info("Line detection node started.")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run detection
        detector = LineDetector(frame)
        edges = detector.preprocess()
        lines = detector.detect_lines(edges)
        output = detector.draw_lines(lines)

        # Convert back to ROS message
        out_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
