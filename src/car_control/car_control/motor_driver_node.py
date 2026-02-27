import rclpy
from rclpy.node import Node
from chassis_control.msg import SetVelocity
from smbus2 import SMBus
import time


I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR = 0x33

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        self.subscription = self.create_subscription(
            SetVelocity,
            '/chassis_control/set_velocity',
            self.callback,
            10
        )

        self.bus = SMBus(1)

        # --- INIT BOARD (like Arduino setup()) ---
        motor_type = 3  # JGB37_520_12V_110RPM
        polarity = 0

        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_TYPE_ADDR, [motor_type])
        time.sleep(0.01)
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, [polarity])

        self.get_logger().info("Motor board initialized")

    def callback(self, msg):

        speed = int(max(-100, min(100, msg.speed)))

        # 4 motors same speed
        motor_speeds = [speed, -speed, speed, -speed]

        try:
            self.bus.write_i2c_block_data(
                I2C_ADDR,
                MOTOR_FIXED_SPEED_ADDR,
                motor_speeds
            )

        except Exception as e:
            self.get_logger().error(f"I2C Write Failed: {e}")

    def destroy_node(self):
        # Stop motors on shutdown
        try:
            self.bus.write_i2c_block_data(
                I2C_ADDR,
                MOTOR_FIXED_SPEED_ADDR,
                [0, 0, 0, 0]
            )
        except Exception as e:
            self.get_logger().error(f"I2C Write Failed during shutdown: {e}")
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
