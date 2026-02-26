from src.motors import MotorController
import time

"""
motors.py

Simple motor/steering controller for a Raspberry Pi (with fallback simulation).
Provides a MotorController that maps steering angle and speed to PWM signals.

- Steering: expected angle range (deg), default -45..45 mapped to 1..2 ms servo pulse (5..10% at 50Hz)
- Speed: expected percent range -100..100 mapped to 1..2 ms ESC pulse (bidirectional, 1.5ms = stop)

API additions:
- create_motor_controller(**kwargs) -> MotorController
- run_interactive(mc=None) -> interactive CLI using provided or internal MotorController
- MotorController supports context manager protocol (with MotorController(...) as mc:)
"""

try:
    import RPi.GPIO as GPIO
    IS_RPI = True
except Exception:
    # Fallback dummy GPIO for non-RPi environments (simulation)
    IS_RPI = False

    class DummyPWM:
        def __init__(self, pin, freq):
            self.pin = pin
            self.freq = freq
            self.duty = 0.0

        def start(self, duty):
            self.duty = duty
            print(f"[SIM] PWM start on pin {self.pin} freq={self.freq}Hz duty={duty:.2f}%")

        def ChangeDutyCycle(self, duty):
            self.duty = duty
            print(f"[SIM] PWM duty on pin {self.pin} -> {duty:.2f}%")

        def stop(self):
            print(f"[SIM] PWM stop on pin {self.pin}")

    class GPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        @staticmethod
        def setmode(x): print(f"[SIM] GPIO setmode({x})")
        @staticmethod
        def setup(pin, mode): print(f"[SIM] GPIO setup(pin={pin}, mode={mode})")
        @staticmethod
        def PWM(pin, freq): return DummyPWM(pin, freq)
        @staticmethod
        def cleanup(): print("[SIM] GPIO cleanup()")


class MotorController:
    """
    Controls a steering servo and an ESC-driven motor via PWM.

    Parameters:
    - steering_pin: GPIO pin for steering servo PWM
    - throttle_pin: GPIO pin for throttle/ESC PWM
    - pwm_freq: PWM frequency in Hz (50Hz typical for servos/ESCs)
    - steer_angle_range: (min_deg, max_deg) input domain
    - steer_pulse_range: (min_duty, max_duty) PWM duty % corresponding to min/max angle
    - speed_range: (-max_percent, +max_percent) domain for speed
    - speed_pulse_range: (min_duty, neutral_duty, max_duty) for -max..0..+max
    """
    def __init__(
        self,
        steering_pin=17,
        throttle_pin=18,
        pwm_freq=50,
        steer_angle_range=(-45.0, 45.0),
        steer_pulse_range=(5.0, 10.0),           # 1ms -> 2ms at 50Hz => 5% .. 10%
        speed_range=100.0,
        speed_pulse_range=(5.0, 7.5, 10.0)       # min, neutral, max duty %
    ):
        self.steering_pin = steering_pin
        self.throttle_pin = throttle_pin
        self.pwm_freq = pwm_freq
        self.steer_min_angle, self.steer_max_angle = steer_angle_range
        self.steer_min_duty, self.steer_max_duty = steer_pulse_range
        self.speed_max = float(speed_range)
        self.speed_min_duty, self.speed_neutral_duty, self.speed_max_duty = speed_pulse_range

        # Setup GPIO / PWM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.steering_pin, GPIO.OUT)
        GPIO.setup(self.throttle_pin, GPIO.OUT)

        self._steer_pwm = GPIO.PWM(self.steering_pin, self.pwm_freq)
        self._throttle_pwm = GPIO.PWM(self.throttle_pin, self.pwm_freq)

        # Start PWMs at neutral positions
        steer_center = self._angle_to_duty(0.0)
        self._steer_pwm.start(steer_center)
        self._throttle_pwm.start(self.speed_neutral_duty)

        # small delay to let servos/ESC arm
        time.sleep(0.05)

    # Context manager support so you can use `with MotorController(...) as mc:`
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
            self.cleanup()
        except Exception:
            pass

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _angle_to_duty(self, angle_deg):
        """Map steering angle (deg) to PWM duty (%)"""
        a = self._clamp(angle_deg, self.steer_min_angle, self.steer_max_angle)
        # linear mapping
        span_angle = self.steer_max_angle - self.steer_min_angle
        if span_angle == 0:
            return (self.steer_min_duty + self.steer_max_duty) / 2.0
        t = (a - self.steer_min_angle) / span_angle
        duty = self.steer_min_duty + t * (self.steer_max_duty - self.steer_min_duty)
        return duty

    def _speed_to_duty(self, speed_percent):
        """Map speed percent (-speed_max..+speed_max) to PWM duty (%) with neutral in middle."""
        s = self._clamp(speed_percent, -self.speed_max, self.speed_max)
        if s == 0:
            return self.speed_neutral_duty
        if s > 0:
            # map 0..max -> neutral..max_duty
            t = s / self.speed_max
            duty = self.speed_neutral_duty + t * (self.speed_max_duty - self.speed_neutral_duty)
        else:
            # map -max..0 -> min_duty..neutral
            t = (s + self.speed_max) / self.speed_max  # s negative -> 0..1
            duty = self.speed_min_duty + t * (self.speed_neutral_duty - self.speed_min_duty)
        return duty

    def set_controls(self, steering_angle_deg, speed_percent):
        """
        Set both steering and throttle.

        - steering_angle_deg: degrees, within steer_angle_range
        - speed_percent: -speed_range..+speed_range (negative for reverse)
        """
        duty_steer = self._angle_to_duty(steering_angle_deg)
        duty_speed = self._speed_to_duty(speed_percent)
        self._steer_pwm.ChangeDutyCycle(duty_steer)
        self._throttle_pwm.ChangeDutyCycle(duty_speed)

    def stop(self):
        """Bring throttle to neutral (stop) and keep steering center."""
        self._throttle_pwm.ChangeDutyCycle(self.speed_neutral_duty)
        self._steer_pwm.ChangeDutyCycle(self._angle_to_duty(0.0))

    def cleanup(self):
        """Stop PWMs and cleanup GPIO"""
        try:
            self._steer_pwm.stop()
            self._throttle_pwm.stop()
        except Exception:
            pass
        GPIO.cleanup()


# Factory function for convenience
def create_motor_controller(**kwargs):
    return MotorController(**kwargs)


# Interactive runner that can be called from another module
def run_interactive(mc=None):
    """
    Run the interactive CLI. If mc is None a MotorController is created and cleaned up here.
    If you pass an existing MotorController, it will not be cleaned up (caller retains control).
    """
    own = False
    if mc is None:
        mc = MotorController()
        own = True

    try:
        print("Enter steering_angle(deg) and speed(percent) separated by space. Ctrl-C to exit.")
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            parts = line.split()
            if len(parts) < 2:
                print("need two numbers: angle speed")
                continue
            try:
                angle = float(parts[0])
                speed = float(parts[1])
            except ValueError:
                print("invalid numbers")
                continue
            mc.set_controls(angle, speed)
    except KeyboardInterrupt:
        pass
    finally:
        if own:
            mc.stop()
            mc.cleanup()


if __name__ == "__main__":
    run_interactive()