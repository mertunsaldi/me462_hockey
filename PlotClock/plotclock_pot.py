import machine, math, time

# ─────────────────── PIN DEFINITIONS ────────────────────────────────────
SERVO_LIFT_PIN  = 2              # GP2
SERVO_LEFT_PIN  = 3              # GP3
SERVO_RIGHT_PIN = 4              # GP4

POT_LEFT_PIN  = 28               # GP28 / ADC2 – X
POT_RIGHT_PIN = 27               # GP27 / ADC1 – Y
POT_LIFT_PIN  = 26               # GP26 / ADC0 – Z

BTN_MODE_PIN   = 8               # GP8   – cycle 0↔1
BTN_ERASE_PIN  = 11              # GP11  – erase area

# ─────────────────── SERVO DRIVER (unchanged) ───────────────────────────
#mimic "servo.writeMicroseconds(...)" via PWM at 50 Hz.
class Servo:
    def __init__(self, pin):
        self.pwm = machine.PWM(machine.Pin(pin))
        self.pwm.freq(50)
        #self._us = 1500

    def writeMicroseconds(self, us):
        #us = max(500, min(2500, us))
        self._us = us
        self.pwm.duty_u16(int(us / 20000 * 65535))

# ─────────────────── SIMPLE BUTTON WITH DEBOUNCE ────────────────────────
class Button:
    def __init__(self, pin, debounce_ms=10):
        self.pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.debounce = debounce_ms
        self._pressed = False
        self._last_ms = time.ticks_ms()
        self.pin.irq(self._irq, machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING)

    def _irq(self, p):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ms) > self.debounce and p.value() == 0:
            self._pressed = True
            self._last_ms = now

    def pressed(self):
        if self._pressed:
            self._pressed = False
            return True
        return False

# ─────────────────── CONSTANTS ─────────────────────────
# adjust these values to have a 90 degree movement (during calibration mode)
# increase these factors to servos rotate more
SERVO_LEFT_FACTOR  = 700
SERVO_RIGHT_FACTOR = 670

SERVO_LEFT_NULL    = 1950
SERVO_RIGHT_NULL   =  815

Z_OFFSET = 230
LIFT0, LIFT1, LIFT2 = 1110+Z_OFFSET, 925+Z_OFFSET, 735+Z_OFFSET
LIFT_SPEED = 2000

L1, L2, L3, L4 = 35.0, 55.1, 13.2, 45.0
O1X, O1Y = 24.0, -25.0
O2X, O2Y = 49.0, -25.0
HOME_X, HOME_Y = 72.2, 45.5

MOVE_DELAY_MS = 0

# ─────────────────── GLOBALS ─────────────────────────────────────────────
currentMode = 1        # 0 = calibration, 1 = manual pot control
servoLift   = 1500.0
lastX, lastY = HOME_X, HOME_Y

# ─────────────────── HARDWARE OBJECTS ───────────────────────────────────
servo_lift  = Servo(SERVO_LIFT_PIN)
servo_left  = Servo(SERVO_LEFT_PIN)
servo_right = Servo(SERVO_RIGHT_PIN)

adc_left  = machine.ADC(POT_LEFT_PIN)
adc_right = machine.ADC(POT_RIGHT_PIN)
adc_lift  = machine.ADC(POT_LIFT_PIN)

btn_mode  = Button(BTN_MODE_PIN)
btn_erase = Button(BTN_ERASE_PIN)

# ─────────────────── SMALL HELPERS ──────────────────────────────────────
def map_range(x, in_lo, in_hi, out_lo, out_hi):
    return out_lo + (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)

def angle_to_pulse(ang, a0=0, a1=180, p0=500, p1=2500):
    return int(map_range(ang, a0, a1, p0, p1))

def acos_clamped(x):
    return math.acos(max(-1, min(1, x)))

def return_angle(a, b, c):
    return acos_clamped((a*a + c*c - b*b) / (2*a*c))

# ─────────────────── KINEMATICS ─────────────────────────────────────────
def set_XY(Tx, Ty):
    time.sleep_ms(1)
    dx, dy = Tx-O1X, Ty-O1Y
    c  = math.sqrt(dx*dx + dy*dy)
    a1 = math.atan2(dy, dx)
    a2 = return_angle(L1, L2, c)

    servo_left.writeMicroseconds(int(((a2 + a1 - math.pi) * SERVO_LEFT_FACTOR) + SERVO_LEFT_NULL))

    a2b = return_angle(L2, L1, c)
    Hx = Tx + L3 * math.cos((a1 - a2b + 0.621) + math.pi)
    Hy = Ty + L3 * math.sin((a1 - a2b + 0.621) + math.pi)

    dx2, dy2 = Hx - O2X, Hy - O2Y
    c2 = math.sqrt(dx2*dx2 + dy2*dy2)
    a1b = math.atan2(dy2, dx2)
    a2c = return_angle(L1, L4, c2)

    servo_right.writeMicroseconds(int(((a1b - a2c) * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL))

def drawTo(pX, pY):
    global lastX, lastY
    dx, dy = pX-lastX, pY-lastY
    steps = max(1, int(7*math.sqrt(dx*dx+dy*dy)))
    for i in range(steps+1):
        #this gets the target x,y -> returns angle inputs of the motors (inverse kin.) -> and runs the motors.
        set_XY(lastX + dx*i/steps, lastY + dy*i/steps) 
        time.sleep_ms(MOVE_DELAY_MS)
    lastX, lastY = pX, pY

def lift(target):
    global servoLift
    step = -1 if servoLift > target else 1
    while servoLift != target:
        servoLift += step
        servo_lift.writeMicroseconds(int(servoLift))
        time.sleep_us(LIFT_SPEED)

def erase():
    goHome()
    lift(LIFT0)
    drawTo(70, HOME_Y); drawTo(65-3, HOME_Y); drawTo(5, HOME_Y)
    drawTo(63-3, 46);   drawTo(63-3, 42);   drawTo(5, 42)
    drawTo(5, 38);      drawTo(63-3,38);    drawTo(63-3,34); drawTo(5,34)
    drawTo(5,29);       drawTo(6,29);       drawTo(65-3,26); drawTo(5,26)
    drawTo(60-3,40);    drawTo(HOME_X, HOME_Y)
    lift(LIFT0)

def goHome():
    lift(LIFT2)
    drawTo(HOME_X, HOME_Y)
    lift(LIFT0)
    time.sleep_ms(500)

# ─────────────────── SETUP ──────────────────────────────────────────────
def setup():
    goHome()  # same startup as before
    time.sleep(2)

# ─────────────────── LOOP ───────────────────────────────────────────────
def loop():
    global currentMode

    # 1) handle buttons ---------------------------------------------------
    if btn_mode.pressed():
        currentMode = (currentMode + 1) & 1   # toggle 0/1
        print("Mode →", currentMode)          # debug

    if btn_erase.pressed():
        erase()

    # 2) run selected mode ------------------------------------------------
    if currentMode == 0:                      # ── Calibration sweep
        drawTo(-3, 29.2)
        time.sleep_ms(500)
        drawTo(74.1, 28)
        time.sleep_ms(500)

    else:                                     # ── Current mode ==1 Manual via pots
        rawX, rawY, rawZ = adc_left.read_u16(), adc_right.read_u16(), adc_lift.read_u16()
        Tx = map_range(rawX, 0, 65535, 0, 120)
        Ty = map_range(rawY, 0, 65535, 0, 100)
        liftDeg = map_range(rawZ, 0, 65535, 0, 180)

        if abs(Tx-lastX) > 0.5 or abs(Ty-lastY) > 0.5:
            set_XY(Tx, Ty)

        servo_lift.writeMicroseconds(angle_to_pulse(liftDeg))

    time.sleep_ms(10)

# ─────────────────── MAIN ───────────────────────────────────────────────
setup()
while True:
    loop()
