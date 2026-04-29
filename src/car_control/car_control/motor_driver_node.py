import rclpy
from rclpy.node import Node
from chassis_control.msg import SetVelocity
from smbus2 import SMBus
import time
import lgpio


I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR = 0x33
MOTOR_ENCODER_TOTAL_ADDR = 0x3C
STEER_CENTER = 1500
STEER_RANGE = 500

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        self.Kp_diff = 0.02   # proportional gain (tune this)
        self.Kp_speed = 0.1   # proportional gain for speed control (tune this)
        self.Ki_speed = 0.01  # integral gain for speed control (tune this)
        self.speed_error_sum = 0   # integral term accumulator for speed control
        self.speed = 0
        self.image_width = 640
        self.max_speed = 80
        self.min_speed = -80
        self.kp_turn = 0.1
        self.dt = 0.1  # time step for integral calculation

        self.prev_encoder = [0, 0, 0, 0]
        self.prev_time = time.time()
        self.timer = self.create_timer(0.1, self.control_loop)

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

        self.h = lgpio.gpiochip_open(4) # Find the correct gpiochip (usually 4)
        self.pin = 17  # BCM pin number for steering servo
        lgpio.gpio_claim_output(self.h, self.pin) # Claim the pin for output
        lgpio.tx_servo(self.h, self.pin, 1500) # Center the servo
        time.sleep(2)

        self.get_logger().info("Motor board initialized")

    def callback(self, msg):

        self.speed = int(max(-100, min(100, msg.speed)))

        # Calculate steering pulse
        steer_pulse = msg.steering_angle
        steer_pulse = max(1000, min(2000, steer_pulse))  # Constrain to valid range
        steer_pulse = int(steer_pulse)

        #print("trying to send servo pulse:", steer_pulse)
        lgpio.tx_servo(self.h, self.pin, steer_pulse) # Use the correct pin number here (17 for BCM) 
        self.get_logger().debug(f"Published: speed={self.speed}, steer_pulse={steer_pulse}")
        self.get_logger().debug("subscribed data: speed=%d, steering_angle=%.2f" % (msg.speed, msg.steering_angle))


    def read_motor_speeds(self):
        try:
            data = self.bus.read_i2c_block_data(
                I2C_ADDR,
                MOTOR_ENCODER_TOTAL_ADDR,
                16
            )

            # convert 4 int32 values
            enc = [
                int.from_bytes(data[0:4],  byteorder='little', signed=True),
                int.from_bytes(data[4:8],  byteorder='little', signed=True),
                int.from_bytes(data[8:12], byteorder='little', signed=True),
                int.from_bytes(data[12:16], byteorder='little', signed=True),
            ]

            now = time.time()
            dt = now - self.prev_time
            if dt <= 0:
                return [0, 0, 0, 0]

            speeds = []
            for i in range(4):
                speeds.append((enc[i] - self.prev_encoder[i]) / dt)

            self.prev_encoder = enc
            self.prev_time = now

            return speeds

        except Exception as e:
            self.get_logger().error(f"Encoder read failed: {e}")
            return [0, 0, 0, 0]

    def control_loop(self):
        speeds = self.read_motor_speeds()

        left = speeds[0]
        right = speeds[2]

        error = left + right
        # proportional correction  (for same speed)
        correction = self.Kp_diff * error 

        avg = (left + right) / 2
        speed_error = self.speed - avg
        self.speed_error_sum += speed_error * self.dt  # accumulate integral error
        self.speed_error_sum = max(min(self.speed_error_sum, 1000), -1000)  # anti-windup for integral term
        
        base = (
            self.Kp_speed * speed_error  +
            self.Ki_speed * self.speed_error_sum
        )


        left_cmd  = base - correction
        right_cmd = base - correction # invert right motor
        right_cmd = -right_cmd

        motor_speeds = [int(left_cmd), int(left_cmd), int(right_cmd), int(right_cmd)]

        try:
            self.get_logger().debug(f"Setting motor speeds: {motor_speeds}")
            self.bus.write_i2c_block_data(
                I2C_ADDR,
                MOTOR_FIXED_SPEED_ADDR,
                motor_speeds
            )

        except Exception as e:
            self.get_logger().error(f"I2C Write Failed: {e}")

        if self.speed is not None:
            self.get_logger().info(f"Target Speed: {self.speed:.2f} | speed: {avg:.2f}  | motor Error: {error:.2f} | Correction: {correction:.2f} | Left Cmd: {left_cmd:.2f} | Right Cmd: {right_cmd:.2f}")
        else:
            self.get_logger().info("Failed to read encoder speed")
        

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
