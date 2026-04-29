import sys
import time
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


class XboxDriveNode(Node):

    def __init__(self):
        super().__init__('xbox_drive_node')

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(
            SetVelocity,
            '/chassis_control/set_velocity',
            10
        )
        
        self.get_logger().info("pigpio initialized and servo centered")
        self.get_logger().info("Initializing Xbox controller...")

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

        # ---------------- Steering ----------------

        steer_axis = self.joystick.get_axis(2)

        # Deadzone for steering (prevents jitter)
        if abs(steer_axis) < 0.05:
            steer_axis = 0.0

        # Convert axis (-1 to 1) → pulse width
        steer_pulse = STEER_CENTER + (steer_axis * STEER_RANGE)

        # HARD clamp to valid servo range (important for lgpio)
        steer_pulse = max(1000, min(2000, steer_pulse))
        steer_pulse = int(steer_pulse)

        self.get_logger().info(f"Speed: {speed} | Steering: {steer_pulse}")

        # Get encoder speeds (placeholder values - replace with actual encoder reads)
        left_encoder = 0  # Replace with actual left encoder reading
        right_encoder = 0  # Replace with actual right encoder reading

        # Synchronize motor speeds based on encoder feedback
        left_speed, right_speed = self.sync_motor_speeds(speed, speed, left_encoder, right_encoder)
        
        # Publish message
        msg = SetVelocity()
        msg.speed = int(speed)
        msg.steering_angle = int(steer_pulse)
        msg.rotation = 0
        self.publisher_.publish(msg)
        print(f"Published: speed={speed}, steer_pulse={steer_pulse}")

        # Move servo
        print("trying to send servo pulse:", steer_pulse)
        #lgpio.tx_servo(self.h, 17, steer_pulse) # Use the correct pin number here (17 for BCM)
        print("servo sent:", steer_pulse)
        


        

    def destroy_node(self):
        # Stop safely
        stop_msg = SetVelocity()
        stop_msg.speed = 0
        stop_msg.angle = 90
        stop_msg.rotate = 0

        self.publisher_.publish(stop_msg)
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



