import pygame
from gpiozero import PWMOutputDevice, Servo
from time import sleep

# Motors
in1 = PWMOutputDevice(17)
in2 = PWMOutputDevice(22)
in3 = PWMOutputDevice(23)
in4 = PWMOutputDevice(24)

ena = PWMOutputDevice(13)
enb = PWMOutputDevice(12)

servo_pin = PWMOutputDevice(18, frequency=50)  # Servo control pin

# Servo
servo = Servo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller connected")

def set_motors(speed, direction):
    if direction == 1:
        # forward
        in1.off()
        in2.on()
        in3.off()
        in4.on()

        ena.value = speed
        enb.value = speed

    elif direction == -1:
        # reverse
        speed = abs(speed)

        in1.on()
        in2.off()
        in3.on()
        in4.off()

        ena.value = speed
        enb.value = speed

    else:
        stop()

def stop():
    servo.value = 0  # straight
    sleep(0.5)
    in1.off()
    in2.off()
    in3.off()
    in4.off()

def drive(sec):
    in1.off()
    in2.on()
    in3.off()
    in4.on()
    sleep(sec)

def set_steering(angle, step=0.04, delay=0.05):
    if angle < -1 or angle > 1:
        raise ValueError("Angle must be between -1 and 1")

    current = servo.value if servo.value is not None else 0

    while abs(current - angle) > 1e-6:
        if current < angle:
            current += step
        else:
            current -= step

        # Clamp to valid range AND target
        current = max(-1, min(1, current))

        # Prevent overshooting target
        if (angle > servo.value and current > angle) or \
           (angle < servo.value and current < angle):
            current = angle

        servo.value = current
        sleep(delay)

def set_servo(angle):
    """
    angle: -1 (full left) → 0 (center) → 1 (full right)
    """
    # Clamp
    angle = max(-1, min(1, angle))

    # Map -1 → 1 to duty cycle 0.05 → 0.10
    duty = 0.075 + 0.025 * angle   # center = 0.075, adjust if needed

    servo_pin.value = duty

def move_servo_smooth_25(target, step=0.002, delay=0.01):
    current = servo_pin.value or 0.075  # start at center
    # Map back to -1..1
    current_angle = (current - 0.075) / 0.025

    while abs(current_angle - target) > 1e-3:
        if target > current_angle:
            current_angle += step
        else:
            current_angle -= step

        # Convert to duty cycle
        servo_pin.value = 0.075 + 0.025 * current_angle
        sleep(delay)

    servo_pin.value = 0.075 + 0.025 * target  # exact final

def move_servo_smooth_18(target, step=0.02, delay=0.01):
    """
    target: -1..1
    step: incremental angle change
    """
    # Get current angle
    current_duty = servo_pin.value or angle_to_duty(0)
    current_angle = ((current_duty * (1_000_000 / 50)) - 500) / 2000 * 2 - 1

    while abs(current_angle - target) > step:
        current_angle += step if target > current_angle else -step
        servo_pin.value = angle_to_duty(current_angle)
        sleep(delay)

    servo_pin.value = angle_to_duty(target)  # final position

def angle_to_duty(angle, min_us=500, max_us=2500, freq=50):
    """
    angle: -1 (full left) → 0 (center) → 1 (full right)
    returns: duty cycle 0..1
    """
    angle = max(-1, min(1, angle))  # clamp

    # Map angle to microseconds
    pulse_us = (angle + 1) / 2 * (max_us - min_us) + min_us

    # Convert microseconds to duty cycle
    period_us = 1_000_000 / freq
    duty = pulse_us / period_us

    return duty




drive_enabled = False

while True:
    pygame.event.pump()

    speed = joystick.get_axis(5)   # the left trigger
    reverse = joystick.get_axis(4) # the right trigger

    # Triggers usually return -1 (not pressed) to 1 (fully pressed), so we remap to 0-1
    speed = (speed + 1) / 2
    reverse = (reverse + 1) / 2

    # Right stick horizontal → steering
    steering = joystick.get_axis(2)

    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:
                drive_enabled = True
                print("Drive ENABLED")

            if event.button == 1:
                drive_enabled = False
                print("Drive DISABLED")
                stop()
    pygame.event.pump()

    # Deadzone (important)
    if abs(speed) < 0.1:
        speed = 0
    if abs(steering) < 0.1:
        steering = 0

    # Apply controls    press a to drive, b to stop 
    steering = steering * 0.5  # reduce sensitivity
    #set_steering(steering)
    #servo_pin.value = angle_to_duty(0)  # for direct servo control

    move_servo_smooth_18(steering)  # for smoother servo movement
    print(f"Speed: {speed:.2f}, Reverse: {reverse:.2f}, Steering: {steering:.2f}")

    total_speed = abs(speed - reverse) # 
    if drive_enabled:
        set_motors(total_speed, 1 if speed > reverse else -1)
    elif not drive_enabled:
        stop()
