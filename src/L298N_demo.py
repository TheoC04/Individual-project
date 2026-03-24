import RPi.GPIO as gpio
from time import sleep
from gpiozero import OutputDevice, Servo

    

in1 = OutputDevice(17)
in2 = OutputDevice(22)
in3 = OutputDevice(23)
in4 = OutputDevice(24)
servo = Servo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)
def stop():
    servo.value = 0  # straight
    sleep(0.5)
    in1.off()
    in2.off()
    in3.off()
    in4.off()

def forward(sec):
    servo.value = 0  # straight
    sleep(0.5)
    in1.off()
    in2.on()
    in3.off()
    in4.on()
    sleep(sec)
    stop()

def reverse(sec):
    servo.value = 0  # straight
    sleep(0.5)
    in1.on()
    in2.off()
    in3.off()
    in4.on()
    sleep(sec)
    stop()

def left_turn(sec):
    servo.value = -1  # left
    sleep(0.5)
    in1.on()
    in2.off()
    in3.on()
    in4.off()
    sleep(sec)
    stop()

def right_turn(sec):
    servo.value = 1  # right
    in1.off()
    in2.on()
    in3.off()
    in4.on()
    sleep(sec)
    stop()

def servo_test():
    for i in range(-10, 11):
        servo.value = i / 10
        sleep(0.5)

def set_steering(angle, step=0.02, delay=0.01):
    if angle < -1 or angle > 1:
        raise ValueError("Angle must be between -1 and 1")

    current = servo.value if servo.value is not None else 0

    while abs(current - angle) > 1e-6:
        if current < angle:
            current += step
        else:
            current -= step

        # 🔒 Clamp to valid range AND target
        current = max(-1, min(1, current))

        # Prevent overshooting target
        if (angle > servo.value and current > angle) or \
           (angle < servo.value and current < angle):
            current = angle

        servo.value = current
        sleep(delay)

def steering_test():
    servo.value = 0  # center
    sleep(1)
    set_steering(-1)  # full left
    sleep(1)
    set_steering(1)   # full right
    sleep(1)
    set_steering(0)   # center
    sleep(1)

seconds = 3
print("Testing servo...")
steering_test()
print("Testing forward...")

sleep(seconds)
print("forward")
forward(seconds)

sleep(seconds-2)
print("right")
right_turn(seconds)

sleep(seconds-2)
sleep(seconds)
print("forward")
forward(seconds)

sleep(seconds-2)
print("right")
right_turn(seconds)
sleep(seconds-2)