import sys
import time
import geometry_msgs
import pygame
import pigpio
import rclpy
from rclpy.node import Node
from chassis_control.msg import SetVelocity
import lgpio

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

        self.subscription = self.create_subscription(
            geometry_msgs.msg.Point,
            '/vision/target_point',
            self.target_point_callback,
            10
        )
        
        # Create timer (30 Hz loop)
        self.timer = self.create_timer(0.033, self.control_loop)
    
    def target_point_callback(self, msg):
        self.get_logger().debug(f"Received target point: ({msg.x}, {msg.y})")
        self.target_x = msg.x
        self.target_y = msg.y

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
        cmd_msg.rotate = 0
        self.publisher_.publish(cmd_msg)
        self.get_logger().debug(f"Published command: speed={cmd_msg.speed}, steering_angle={cmd_msg.steering_angle}")        
        

    def destroy_node(self):
        # Stop safely
        stop_msg = SetVelocity()
        stop_msg.speed = 0
        stop_msg.angle = 90
        stop_msg.rotate = 0

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



