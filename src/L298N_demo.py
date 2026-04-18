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

    gpio.setup(25, gpio.OUT)
    servo = gpio.PWM(25, 50)  # 50 Hz for servo
    servo.start(7.5)  # neutral position

def forward(sec):
    init()
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(sec)
    gpio.cleanup() 

def reverse(sec):
    init()
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(sec)
    gpio.cleanup()

def left_turn(sec):
    init()
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(sec)
    gpio.cleanup()
    
def right_turn(sec):
    init()
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(sec)
    gpio.cleanup()

def set_angle(duty):
    servo.ChangeDutyCycle(duty)
    time.sleep(0.3)

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