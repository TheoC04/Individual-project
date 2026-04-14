import lgpio
import time

h = lgpio.gpiochip_open(4)
pin = 17  # BCM pin number for steering servo

lgpio.gpio_claim_output(h, pin)

while True:
    lgpio.tx_servo(h, pin, 1000)
    time.sleep(1)

    lgpio.tx_servo(h, pin, 1500)
    time.sleep(1)

    lgpio.tx_servo(h, pin, 2000)
    time.sleep(1)