import machine
import math
import time

# ─────────────────────────────────────────────────────────────────────────────
#                              PIN DEFINITIONS
# ─────────────────────────────────────────────────────────────────────────────
SERVO_LIFT_PIN  = 2         # GP2
SERVO_LEFT_PIN  = 3         # GP3
SERVO_RIGHT_PIN = 4         # GP4

POT_LEFT_PIN  = 28          # GP28 / ADC2  – X axis
POT_RIGHT_PIN = 27          # GP27 / ADC1  – Y axis
POT_LIFT_PIN  = 26          # GP26 / ADC0  – Z (pen lift)

# ─────────────────────────────────────────────────────────────────────────────
#                       SERVO DRIVER   (unchanged)
# ─────────────────────────────────────────────────────────────────────────────
class Servo:
    def __init__(self, pin_num):
        self.pwm = machine.PWM(machine.Pin(pin_num))
        self.pwm.freq(50)            # 50 Hz
        self._us = 1500

    def writeMicroseconds(self, us):
        if us < 500:  us = 500
        if us > 2500: us = 2500
        duty = int((us / 20000.0) * 65535.0)
        self.pwm.duty_u16(duty)
        self._us = us

# ─────────────────────────────────────────────────────────────────────────────
#                     ORIGINAL CONSTANTS / VARIABLES
# ─────────────────────────────────────────────────────────────────────────────
SERVO_LEFT_FACTOR  = 630
SERVO_RIGHT_FACTOR = 640

Z_OFFSET = 230
LIFT0 = 1110 + Z_OFFSET
LIFT1 = 925  + Z_OFFSET
LIFT2 = 735  + Z_OFFSET

SERVO_LEFT_NULL  = 1950
SERVO_RIGHT_NULL = 815

WISHY = 3
LIFT_SPEED = 2000

L1 = 35.0
L2 = 55.1
L3 = 13.2
L4 = 45.0

O1X = 24.0
O1Y = -25.0
O2X = 49.0
O2Y = -25.0

HOME_X = 72.2
HOME_Y = 45.5

SCALE = 0.9
MOVE_DELAY_MS = 2

servoLift = 1500.0
lastX = HOME_X
lastY = HOME_Y

# ─────────────────────────────────────────────────────────────────────────────
#                         SERVO OBJECTS
# ─────────────────────────────────────────────────────────────────────────────
servo_lift  = Servo(SERVO_LIFT_PIN)
servo_left  = Servo(SERVO_LEFT_PIN)
servo_right = Servo(SERVO_RIGHT_PIN)

# ─────────────────────────────────────────────────────────────────────────────
#                           ADC OBJECTS
# ─────────────────────────────────────────────────────────────────────────────
adc_left  = machine.ADC(POT_LEFT_PIN)
adc_right = machine.ADC(POT_RIGHT_PIN)
adc_lift  = machine.ADC(POT_LIFT_PIN)

# quick helper
def map_range(x, in_lo, in_hi, out_lo, out_hi):
    return out_lo + (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)

# ─────────────────────────────────────────────────────────────────────────────
#                    Mapping Function for Lift Servo  (unchanged)
# ─────────────────────────────────────────────────────────────────────────────
def angle_to_pulse(angle, min_angle=0, max_angle=180, min_pulse=500, max_pulse=2500):
    return int(min_pulse + (angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle))

# ─────────────────────────────────────────────────────────────────────────────
#                                SETUP
# ─────────────────────────────────────────────────────────────────────────────
def setup():
    # move to home as before
    lift(LIFT2)
    drawTo(HOME_X, HOME_Y)
    lift(LIFT0)
    time.sleep(2)

# ─────────────────────────────────────────────────────────────────────────────
#                                 LOOP
# ─────────────────────────────────────────────────────────────────────────────
def loop():
    global lastX, lastY

    # ---- 1. read potentiometers ------------------------------------------------
    rawX  = adc_left.read_u16()     # 0–65535
    rawY  = adc_right.read_u16()
    rawZ  = adc_lift.read_u16()

    # scale to same coordinate units the original code expects
    targetX = map_range(rawX, 0, 65535,  0, 120)   # adjust limits if desired
    targetY = map_range(rawY, 0, 65535,  0, 100)
    liftDeg = map_range(rawZ, 0, 65535,  0, 180)

    # ---- 2. move only if position changed by ≥0.5 mm to tame ADC noise ---------
    if abs(targetX - lastX) > 0.5 or abs(targetY - lastY) > 0.5:
        set_XY(targetX, targetY)
        lastX, lastY = targetX, targetY

    # ---- 3. lift servo ---------------------------------------------------------
    servo_lift.writeMicroseconds(angle_to_pulse(liftDeg))

    time.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#                           LIFT FUNCTION (unchanged)
# ─────────────────────────────────────────────────────────────────────────────
def lift(lift_target):
    global servoLift
    if servoLift >= lift_target:
        while servoLift >= lift_target:
            servoLift -= 1
            servo_lift.writeMicroseconds(int(servoLift))
            time.sleep_us(LIFT_SPEED)
    else:
        while servoLift <= lift_target:
            servoLift += 1
            servo_lift.writeMicroseconds(int(servoLift))
            time.sleep_us(LIFT_SPEED)

# ─────────────────────────────────────────────────────────────────────────────
#                   ERASE / DRAW / MISC   (unchanged)
# ─────────────────────────────────────────────────────────────────────────────
def erase():
    goHome()
    lift(LIFT0)
    drawTo(70, HOME_Y)
    drawTo(65 - WISHY, HOME_Y)
    drawTo(65 - WISHY, HOME_Y)
    drawTo(5, HOME_Y)
    drawTo(5, HOME_Y)
    drawTo(63 - WISHY, 46)
    drawTo(63 - WISHY, 42)
    drawTo(5, 42)
    drawTo(5, 38)
    drawTo(63 - WISHY, 38)
    drawTo(63 - WISHY, 34)
    drawTo(5, 34)
    drawTo(5, 29)
    drawTo(6, 29)
    drawTo(65 - WISHY, 26)
    drawTo(5, 26)
    drawTo(60 - WISHY, 40)
    drawTo(HOME_X, HOME_Y)
    lift(LIFT0)

def goHome():
    lift(LIFT2)
    drawTo(HOME_X, HOME_Y)
    lift(LIFT0)
    time.sleep_ms(500)

def bogenUZS(bx, by, radius, start, ende, sqee):
    inkr = -0.05
    count = 0
    while (start + count) > ende:
        drawTo(
            sqee * radius * math.cos(start + count) + bx,
            radius * math.sin(start + count) + by
        )
        count += inkr

def bogenGZS(bx, by, radius, start, ende, sqee):
    inkr = 0.05
    count = 0
    while (start + count) <= ende:
        drawTo(
            sqee * radius * math.cos(start + count) + bx,
            radius * math.sin(start + count) + by
        )
        count += inkr

def drawTo(pX, pY):
    global lastX, lastY
    dx = pX - lastX
    dy = pY - lastY
    c = int(math.floor(7 * math.sqrt(dx * dx + dy * dy)))   # ← math.hypot avoided
    if c < 1:
        c = 1
    for i in range(c+1):
        ix = lastX + (i * dx / c)
        iy = lastY + (i * dy / c)
        set_XY(ix, iy)
        time.sleep_ms(MOVE_DELAY_MS)
    lastX = pX
    lastY = pY

def acos_clamped(x):
    if x >  1: x =  1
    if x < -1: x = -1
    return math.acos(x)

def return_angle(a, b, c):
    return acos_clamped((a*a + c*c - b*b) / (2*a*c))

def set_XY(Tx, Ty):
    time.sleep_ms(1)
    dx = Tx - O1X
    dy = Ty - O1Y

    c  = math.sqrt(dx*dx + dy*dy)
    a1 = math.atan2(dy, dx)
    a2 = return_angle(L1, L2, c)

    servo_left.writeMicroseconds(
        int(((a2 + a1 - math.pi) * SERVO_LEFT_FACTOR) + SERVO_LEFT_NULL)
    )

    # right‑hand linkage
    a2b = return_angle(L2, L1, c)
    Hx = Tx + L3 * math.cos((a1 - a2b + 0.621) + math.pi)
    Hy = Ty + L3 * math.sin((a1 - a2b + 0.621) + math.pi)

    dx2 = Hx - O2X
    dy2 = Hy - O2Y
    c2  = math.sqrt(dx2*dx2 + dy2*dy2)
    a1b = math.atan2(dy2, dx2)
    a2c = return_angle(L1, L4, c2)

    servo_right.writeMicroseconds(
        int(((a1b - a2c) * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL)
    )

# ─────────────────────────────────────────────────────────────────────────────
#                                MAIN
# ─────────────────────────────────────────────────────────────────────────────
setup()
while True:
    loop()
    time.sleep_ms(10)
