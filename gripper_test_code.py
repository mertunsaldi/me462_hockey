from machine import Pin, PWM
from time import sleep

# 50 Hz → 20 ms period
FREQ      = 50
MIN_DUTY  = 1638    # 0.5 ms pulse  → 0.5/20 * 65535
MAX_DUTY  = 8192    # 2.5 ms pulse  → 2.5/20 * 65535

# PWM on GP3
servo = PWM(Pin(3))
servo.freq(FREQ)

def set_angle(angle):
    # clamp to safe range
    angle = max(0, min(180, angle))
    # map 0–180° → MIN_DUTY–MAX_DUTY
    duty = int((angle / 180) * (MAX_DUTY - MIN_DUTY) + MIN_DUTY)
    servo.duty_u16(duty)

while True:
    set_angle(65)
    sleep(4)        # give it a second to get to 80°
    set_angle(130)
    sleep(4)        # give it a second to get to 120°
