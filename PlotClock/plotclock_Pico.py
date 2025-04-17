import machine
import math
import time

###############################################################################
#                           PIN DEFINITIONS
###############################################################################
SERVO_LIFT_PIN = 2
SERVO_LEFT_PIN = 3
SERVO_RIGHT_PIN = 4

ENCODER_L_PIN1    = 6
ENCODER_L_PIN2    = 7
ENCODER_L_BTN_PIN = 8

ENCODER_R_PIN1    = 9
ENCODER_R_PIN2    = 10
ENCODER_R_BTN_PIN = 11

ENCODER_LIFT_PIN1    = 12
ENCODER_LIFT_PIN2    = 13
ENCODER_LIFT_INIT    = 70  # starting position (in degrees)

###############################################################################
#                       SERVO REPLACEMENT FOR "ESP32Servo"
###############################################################################
# We’ll mimic "servo.writeMicroseconds(...)" via PWM at 50 Hz.
class Servo:
    def __init__(self, pin_num):
        self.pwm = machine.PWM(machine.Pin(pin_num))
        self.pwm.freq(50)          # 50 Hz for standard servos
        self._us = 1500            # track last written microseconds

    def writeMicroseconds(self, us):
        # Clamp to safe range if needed:
        if us < 500:  us = 500
        if us > 2500: us = 2500
        self._us = us
        # Convert microseconds to 16-bit duty:
        duty = int((us / 20000.0) * 65535.0)
        self.pwm.duty_u16(duty)

###############################################################################
#                      ENCODER REPLACEMENT FOR "Encoder.h" 
#         (INTERRUPT-BASED QUADRATURE DECODING, ARDUINO-STYLE)
###############################################################################
class InterruptEncoder:
    """
    A simple quadrature encoder class using interrupts on the Pico.
    Provides a 'read()' method that returns the raw count,
    and a 'write(value)' method to set the count (like encoder.write(...) in Arduino).
    """
    def __init__(self, pinA, pinB):
        self.pinA = machine.Pin(pinA, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pinB = machine.Pin(pinB, machine.Pin.IN, machine.Pin.PULL_UP)

        # current 2-bit state of the encoder
        self._state = (self.pinA.value() << 1) | self.pinB.value()
        self._position = 0

        # Attach interrupts on both edges
        self.pinA.irq(self._callback, machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING)
        self.pinB.irq(self._callback, machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING)

    def _callback(self, pin):
        new_state = (self.pinA.value() << 1) | self.pinB.value()
        if new_state != self._state:
            # quadrature direction logic
            if ((self._state == 0 and new_state == 1) or
                (self._state == 1 and new_state == 3) or
                (self._state == 3 and new_state == 2) or
                (self._state == 2 and new_state == 0)):
                self._position += 1
            else:
                self._position -= 1
            self._state = new_state

    def read(self):
        return self._position

    def write(self, value):
        self._position = value

###############################################################################
#                     BUTTON REPLACEMENT FOR "Button.h"
###############################################################################
class Button:
    """
    Mimics the "Button" library usage:
      - constructor (pin, debounce_ms)
      - begin()
      - pressed() to check if it was pressed since last check
    Uses a simple interrupt-based approach with minimal debounce logic.
    """
    def __init__(self, pin_num, debounce_ms=10):
        self.pin = machine.Pin(pin_num, machine.Pin.IN, machine.Pin.PULL_UP)
        self.debounce_ms = debounce_ms
        self._last_val = self.pin.value()
        self._pressed_flag = False
        self._last_change_time = time.ticks_ms()

        # Set an interrupt on both edges:
        self.pin.irq(self._callback, machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING)

    def _callback(self, p):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_change_time) > self.debounce_ms:
            val = p.value()
            if val == 0:
                # pressed (active low)
                self._pressed_flag = True
            self._last_val = val
            self._last_change_time = now

    def begin(self):
        # mimic the original code's usage "leftEncoderBtn.begin();"
        pass

    def pressed(self):
        if self._pressed_flag:
            self._pressed_flag = False
            return True
        return False

###############################################################################
#                          ORIGINAL CONSTANTS / VARIABLES
###############################################################################
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

# Global servo position for lift (in microseconds for servo, initially 1500)
servoLift = 1500.0
lastX = HOME_X
lastY = HOME_Y

# 0: Calibration, 1: Encoder control (we remove auto mode)
currentMode = 1
print("currentMode: ", currentMode)
###############################################################################
#                          SERVO OBJECTS
###############################################################################
servo_lift  = Servo(SERVO_LIFT_PIN)
servo_left  = Servo(SERVO_LEFT_PIN)
servo_right = Servo(SERVO_RIGHT_PIN)

###############################################################################
#                          ENCODER OBJECTS
###############################################################################
encoderL     = InterruptEncoder(ENCODER_L_PIN1,   ENCODER_L_PIN2)
encPosL      = 0
encLastPosL  = -1

encoderR     = InterruptEncoder(ENCODER_R_PIN1,   ENCODER_R_PIN2)
encPosR      = 0
encLastPosR  = -1

encoderLift  = InterruptEncoder(ENCODER_LIFT_PIN1, ENCODER_LIFT_PIN2)
encPosLift   = 0
encLastPosLift = -1

###############################################################################
#                        BUTTON OBJECTS
###############################################################################
leftEncoderBtn  = Button(ENCODER_L_BTN_PIN, 10)
rightEncoderBtn = Button(ENCODER_R_BTN_PIN, 10)

###############################################################################
#                  Mapping Function for Lift Servo
###############################################################################
def angle_to_pulse(angle, min_angle=0, max_angle=180, min_pulse=500, max_pulse=2500):           #you should calibrate this for proper lift
    # Convert an angle (in degrees) to a pulse width in microseconds.
    return int(min_pulse + (angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle))

###############################################################################
#                             SETUP() EQUIVALENT
###############################################################################
def setup():
    global currentMode

    # Set initial encoder positions:
    encoderL.write(int(HOME_X * 4))
    encoderR.write(int(HOME_Y * 4))
    encoderLift.write(int(ENCODER_LIFT_INIT * 4))

    # start the button objects
    leftEncoderBtn.begin()
    rightEncoderBtn.begin()

    # If not in calibration mode (currentMode != 0), move to the home position:
    if currentMode != 0:
        lift(LIFT2)
        drawTo(HOME_X, HOME_Y)
        lift(LIFT0)

    time.sleep(2)  # delay of 2000 ms

###############################################################################
#                            LOOP() EQUIVALENT
###############################################################################
def loop():
    global currentMode

    handleEncoderBtns()

    # Calibration mode
    if currentMode == 0:
        drawTo(-3, 29.2)
        time.sleep_ms(500)
        drawTo(74.1, 28)
        time.sleep_ms(500)
    # Manual control mode
    elif currentMode == 1:
        if updateEncoder(encoderL, 'L'):
            set_XY(encPosL, encPosR)
        if updateEncoder(encoderR, 'R'):
            set_XY(encPosL, encPosR)
        if updateEncoder(encoderLift, 'Lift'):
            # Map the lift encoder value (in degrees) to microseconds.
            pulse = angle_to_pulse(encPosLift)
            servo_lift.writeMicroseconds(pulse)

###############################################################################
#                handleEncoderBtns() -- SAME LOGIC AS ORIGINAL
###############################################################################
def handleEncoderBtns():
    global currentMode
    if leftEncoderBtn.pressed():
        currentMode += 1
        if currentMode == 2:
            currentMode = 0
        print("Current mode_handle: ", currentMode)
    if rightEncoderBtn.pressed():
        erase()

###############################################################################
#            updateEncoder() -- SAME LOGIC, ADAPTED TO PYTHON
###############################################################################
def updateEncoder(enc, which):
    """
    Mimics:
      bool updateEncoder(Encoder encoder, int &encPos, int &encLastPos)
    'which' is 'L','R','Lift' to pick the correct globals.
    """
    global encPosL, encLastPosL
    global encPosR, encLastPosR
    global encPosLift, encLastPosLift

    newPos = enc.read() // 4  # Dividing by 4 as in the original code.
    changed = False

    if which == 'L':
        if newPos != encLastPosL:
            encPosL = newPos
            encLastPosL = newPos
            changed = True
    elif which == 'R':
        if newPos != encLastPosR:
            encPosR = newPos
            encLastPosR = newPos
            changed = True
    elif which == 'Lift':
        if newPos != encLastPosLift:
            encPosLift = newPos
            encLastPosLift = newPos
            changed = True
    return changed

###############################################################################
#             LIFT FUNCTION (Same Logic as Original)
###############################################################################
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

###############################################################################
#                           ERASE / DRAW / MISC FUNCTIONS
###############################################################################
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
    c = int(math.floor(7 * math.sqrt(dx * dx + dy * dy)))
    if c < 1:
        c = 1
    for i in range(c+1):
        ix = lastX + (i * dx / c)
        iy = lastY + (i * dy / c)
        set_XY(ix, iy)
        time.sleep_ms(MOVE_DELAY_MS)
    lastX = pX
    lastY = pY

def return_angle(a, b, c):
    return math.acos((a*a + c*c - b*b) / (2*a*c))

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

    # Calculate joint arm point for the right servo.
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

###############################################################################
#                         MAIN BOOTSTRAP
###############################################################################
setup()

while True:
    loop()
    time.sleep_ms(10)
