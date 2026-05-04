
import rclpy
from rclpy.node import Node
from messages.msg import SetVelocity
from geometry_msgs.msg import Point, PointStamped

# -------------------
# CONFIG
# -------------------
MAX_SPEED = 80
STEER_CENTER = 1500
STEER_RANGE = 500
GPIO_PIN = 17  # BCM pin number for steering servo 


'''def get_gpiochip():
    for chip in range(0, 10):
        try:
            h = lgpio.gpiochip_open(chip)
            lgpio.gpiochip_close(h)
            return chip
        except:
            pass
    raise RuntimeError("No usable gpiochip found")'''


class TargetPointControlNode(Node):

    def __init__(self):
        super().__init__('target_point_control_node')

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(
            SetVelocity,
            '/chassis_control/set_velocity',
            10
        )
        self.subscription.qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            PointStamped,
            '/vision/target_point',
            self.target_point_callback,
            10
        )
        
        # Create timer (30 Hz loop)
        self.timer = self.create_timer(0.033, self.control_loop)

        self.image_width = 640  # Assuming a 640px wide image from vision
        self.max_speed = MAX_SPEED
        self.min_speed = 20  # Minimum speed to ensure movement 
        self.kp_turn = 0.5  # Proportional gain for turning control

    
    def target_point_callback(self, msg):
        self.get_logger().info(f"Received target point: ({msg.point.x}, {msg.point.y})")
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.header = msg.header
        t_capture = rclpy.time.Time.from_msg(msg.header.stamp)
        t_now = self.get_clock().now()
        delay = (t_now - t_capture).nanoseconds * 1e-9
        self.get_logger().info(f"Target point message latency: {delay:.3f} seconds")

    def control_loop(self):
        if not hasattr(self, 'target_x') or not hasattr(self, 'target_y'):
            return  # No target point received yet
    
        image_center = self.image_width / 2
        max_speed = self.max_speed
        min_speed = self.min_speed
        kp_turn = self.kp_turn

        # --- error (normalised -1 to 1) ---
        error_x = (self.target_x - image_center) / image_center

        # --- turning control ---
        turn = kp_turn * error_x

        # --- speed reduction based on how far off-centre we are ---
        # |error_x| = 0 → full speed
        # |error_x| = 1 → minimum speed
        speed = max_speed * (1 - abs(error_x))

        # enforce minimum speed so robot still moves
        speed = max(speed, min_speed)

        # --- publish command ---
        cmd_msg = SetVelocity()
        cmd_msg.speed = int(speed)
        cmd_msg.steering_angle = int(STEER_CENTER + turn * STEER_RANGE)
        cmd_msg.rotation = 0
        cmd_msg.header.stamp = self.header.stamp  # Use the same timestamp as the target point message
        self.publisher_.publish(cmd_msg)
        self.get_logger().debug(f"Published command: speed={cmd_msg.speed}, steering_angle={cmd_msg.steering_angle}")       
        

    def destroy_node(self):
        # Stop safely
        stop_msg = SetVelocity()
        stop_msg.speed = 0
        stop_msg.steering_angle = 90
        stop_msg.rotation = 0

        self.publisher_.publish(stop_msg)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetPointControlNode()
    print("Target Point Control Node started. Waiting for target points...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("Target Point Control Node shutdown gracefully.")


if __name__ == '__main__':
    main()



