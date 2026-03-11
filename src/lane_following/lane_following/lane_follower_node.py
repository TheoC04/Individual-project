import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from chassis_control.msg import SetVelocity  # replace with your ROS 2 package
from lane_following.lane_following import LaneDetection
import cv2

class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_follower_node')
        self.get_logger().info("LaneFollowerNode initialized")
        
        # Publishers
        self.cmd_pub = self.create_publisher(SetVelocity, '/setvelocity', 10)
        self.cte_pub = self.create_publisher(Float32, '/lane_cte', 10)
        self.debug_image_pub = self.create_publisher(Image, '/lane_debug_image', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        self.bridge = CvBridge()
        self.lane_detector = LaneDetection()

        # PID parameters
        self.Kp = 0.01
        self.Kd = 0.05
        self.last_cte = 0.0

        # Robot motion constraints
        self.max_speed = 0.3
        self.max_steering = 2.0
        self.max_rotation = 2.0

        self.counter = 1
        self.debug = True

    def camera_callback(self, msg):
        # Frame skipping
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        cte, angle, final_img = self.lane_detector.processImage(cv_image)

        cmd = SetVelocity()

        if cte is not None:
            steering = self.Kp * cte + self.Kd * (cte - self.last_cte)
            self.last_cte = cte
            steering = max(-self.max_steering, min(self.max_steering, steering))

            cmd.speed = int(self.max_speed * 1000)          # scaled to preserve decimals
            cmd.steering_angle = int(steering * 1000)      # scaled
            cmd.rotation = int(steering * 1000)            # scaled
        else:
            cmd.speed = 0
            cmd.steering_angle = 0
            cmd.rotation = 0

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Published command: speed={cmd.speed/1000:.3f}, steering_angle={cmd.steering_angle/1000:.3f}, rotation={cmd.rotation/1000:.3f}")

        if cte is not None:
            self.cte_pub.publish(Float32(data=float(cte)))
            self.get_logger().info(f"Published CTE: {cte:.3f}")

        if final_img is not None and self.debug:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(final_img, encoding='bgr8')
                self.debug_image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"CV Bridge error: {e}")

            cv2.imshow("Lane Detection", final_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()