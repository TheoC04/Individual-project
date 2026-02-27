import sys
import time
import pygame
import pigpio
import rclpy
from rclpy.node import Node
from chassis_control.msg import SetVelocity

# -------------------
# CONFIG
# -------------------
MAX_SPEED = 80
STEER_CENTER = 1500
STEER_RANGE = 500
GPIO_PIN = 12


class XboxDriveNode(Node):

    def __init__(self):
        super().__init__('xbox_drive_node')

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(
            SetVelocity,
            '/chassis_control/set_velocity',
            10
        )

        # Setup pigpio
        '''
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("pigpio not running! Run: sudo pigpiod")
            sys.exit()

        self.pi.set_servo_pulsewidth(GPIO_PIN, STEER_CENTER)
        '''

        # Setup pygame joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No controller detected")
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(f"Connected: {self.joystick.get_name()}")

        # Create timer (30 Hz loop)
        self.timer = self.create_timer(0.033, self.control_loop)

    def control_loop(self):

        pygame.event.pump()

        forward_trigger = self.joystick.get_axis(4)
        backward_trigger = self.joystick.get_axis(5)

        # Convert from (-1 to 1) → (0 to 1)
        forward = (forward_trigger + 1) / 2
        backward = (backward_trigger + 1) / 2

        # Final speed
        speed = int((forward - backward) * MAX_SPEED)

        # Right stick horizontal → steering
        steer_axis = self.joystick.get_axis(2)

        steer_pulse = int(STEER_CENTER + steer_axis * STEER_RANGE)

        steer_pulse = max(
            STEER_CENTER - STEER_RANGE,
            min(STEER_CENTER + STEER_RANGE, steer_pulse)
        )

        # Publish message
        msg = SetVelocity()
        msg.speed = speed
        msg.steering_angle = steer_pulse
        msg.rotation = 0

        self.publisher_.publish(msg)
        print(f"Published: speed={speed}, steer_pulse={steer_pulse}")

        # Move servo
        # self.pi.set_servo_pulsewidth(GPIO_PIN, steer_pulse)

        self.get_logger().info(
            f"Speed: {speed} | Steering: {steer_pulse}"
        )

    def destroy_node(self):
        # Stop safely
        stop_msg = SetVelocity()
        stop_msg.speed = 0
        stop_msg.angle = 90
        stop_msg.rotate = 0

        self.publisher_.publish(stop_msg)
        #self.pi.set_servo_pulsewidth(GPIO_PIN, STEER_CENTER)
        self.pi.stop()
        pygame.quit()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XboxDriveNode()
    print("Controller node started. Use the Xbox controller to drive. Press Ctrl+C to exit.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("Controller node shutdown gracefully.")


if __name__ == '__main__':
    main()



