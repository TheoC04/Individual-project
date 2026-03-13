import rclpy
from rclpy.node import Node
import cv2
from image_transport import ImageTransport
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class USBCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))  # Set YUYV format (from MJPG) for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open USB camera")
            raise RuntimeError("Camera failed")

        self.timer = self.create_timer(0.033, self.capture_frame)

    def capture_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Frame capture failed")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()