import machine
import math
import time
import sys
import select              # USB‑serial support

# ─────────────────── Helper: safe println ──────────────────────────────
def _println(*args, **kwargs):
    """Start on a fresh line, print, and flush if flush() exists."""
    sys.stdout.write("\r\n")
    print(*args, **kwargs)
    if hasattr(sys.stdout, "flush"):
        sys.stdout.flush()

# ─────────────────── PIN DEFINITIONS ───────────────────────────────────
SERVO_LEFT_PIN  = 3   # GP3
SERVO_RIGHT_PIN = 4   # GP4

POT_LEFT_PIN  = 28    # GP28 / ADC2 – X
POT_RIGHT_PIN = 27    # GP27 / ADC1 – Y

# ─────────────────── SERVO DRIVER ──────────────────────────────────────
class Servo:
    def __init__(self, pin):
        self.pwm = machine.PWM(machine.Pin(pin))
        self.pwm.freq(50)

    def writeMicroseconds(self, us):
        self.pwm.duty_u16(int(us / 20000 * 65535))

# ─────────────────── SIMPLE BUTTON WITH DEBOUNCE ───────────────────────
class Button:
    def __init__(self, pin, debounce_ms=10):
        self.pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.debounce = debounce_ms
        self._pressed = False
        self._last_ms = time.ticks_ms()
        self.pin.irq(self._irq,
                     machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING)

    def _irq(self, _):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ms) > self.debounce \
                and self.pin.value() == 0:
            self._pressed = True
            self._last_ms = now

    def pressed(self):
        if self._pressed:
            self._pressed = False
            return True
        return False

# ─────────────────── CONSTANTS ─────────────────────────────────────────
SERVO_LEFT_FACTOR  = 700
SERVO_RIGHT_FACTOR = 670

SERVO_LEFT_NULL  = 1950
SERVO_RIGHT_NULL =  815

Z_OFFSET = 230

L1 = 35.0
L2 = 55.1
L3 = 13.2
L4 = 45.0

O1X, O1Y = 24.0, -25.0
O2X, O2Y = 49.0, -25.0
HOME_X, HOME_Y = 72.2, 45.5

MOVE_DELAY_MS = 0

# ─────────────────── POT‑CONTROL PARAMETERS ────────────────────────────
FILTER_ALPHA = 0.1
DEADBAND_MM  = 2
X_MIN, X_MAX = 0.0, 100.0
Y_MIN, Y_MAX = 20.0, 80.0

# ─────────────────── GLOBALS ────────────────────────────────────────────
currentMode = 4          # 0=cal,1=manual,2=demo,3=home‑loop,4=command‑only
mode4_init  = False
lastX, lastY = HOME_X, HOME_Y

# ─────────────────── HARDWARE OBJECTS ──────────────────────────────────
servo_left  = Servo(SERVO_LEFT_PIN)
servo_right = Servo(SERVO_RIGHT_PIN)

adc_left  = machine.ADC(POT_LEFT_PIN)
adc_right = machine.ADC(POT_RIGHT_PIN)

rawX_filt = adc_left.read_u16()
rawY_filt = adc_right.read_u16()

# ─────────────────── USB‑SERIAL (non‑blocking) ─────────────────────────
_serial_poll = select.poll()
_serial_poll.register(sys.stdin, select.POLLIN)
_serial_buf = bytearray()

# ─────────────────── SMALL HELPERS ─────────────────────────────────────
def map_range(x, in_lo, in_hi, out_lo, out_hi):
    return out_lo + (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)

def angle_to_pulse(angle_deg, p0=500, p1=2500):
    return int(map_range(angle_deg, 0, 180, p0, p1))

def acos_clamped(x):
    return math.acos(max(-1.0, min(1.0, x)))

def return_angle(a, b, c):
    return acos_clamped((a*a + c*c - b*b) / (2.0 * a * c))

# ─────────────────── KINEMATICS ────────────────────────────────────────
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

    a2b = return_angle(L2, L1, c)
    Hx  = Tx + L3 * math.cos((a1 - a2b + 0.621) + math.pi)
    Hy  = Ty + L3 * math.sin((a1 - a2b + 0.621) + math.pi)

    dx2 = Hx - O2X
    dy2 = Hy - O2Y
    c2  = math.sqrt(dx2*dx2 + dy2*dy2)
    a1b = math.atan2(dy2, dx2)
    a2c = return_angle(L1, L4, c2)

    servo_right.writeMicroseconds(
        int(((a1b - a2c) * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL)
    )

def drawTo(pX, pY):
    global lastX, lastY
    dx = pX - lastX
    dy = pY - lastY
    steps = max(1, int(7 * math.sqrt(dx*dx + dy*dy)))
    for i in range(steps + 1):
        set_XY(lastX + dx * i / steps,
               lastY + dy * i / steps)
        time.sleep_ms(MOVE_DELAY_MS)
    lastX, lastY = pX, pY

def goHome():
    drawTo(HOME_X, HOME_Y)
    time.sleep_ms(500)

# ─────────────────── SERIAL PARSER & SERVICE ───────────────────────────
def _parse_cmd(line):
    """
    Supported commands:
      setxy  x  y
      drawto x  y
      gohome
      mode   n
    """
    global currentMode, mode4_init

    _println("CMD:", line)

    parts = line.strip().lower().split()
    if not parts:
        return

    try:
        cmd = parts[0]

        if cmd == "setxy" and len(parts) == 3:
            Tx = float(parts[1])
            Ty = float(parts[2])
            set_XY(Tx, Ty)
            # ——— update current position for the next drawTo() ———
            global lastX, lastY
            lastX, lastY = Tx, Ty

        elif cmd == "drawto" and len(parts) == 3:
            drawTo(float(parts[1]), float(parts[2]))

        elif cmd == "gohome":
            goHome()

        elif cmd == "mode" and len(parts) == 2:
            currentMode = int(parts[1]) & 7
            mode4_init = False
            _println("Mode →", currentMode)

        else:
            _println("ERR: bad cmd →", line)

    except Exception as e:
        _println("ERR:", e)

def _service_serial():
    """Poll USB console without blocking."""
    global _serial_buf

    for fd, ev in _serial_poll.poll(0):
        if ev & select.POLLIN:
            ch = sys.stdin.read(1)
            if not ch or ch == '\r':
                continue
            if ch == '\n':
                try:
                    _parse_cmd(_serial_buf.decode())
                finally:
                    _serial_buf = bytearray()
            else:
                _serial_buf += ch.encode()

# ─────────────────── SETUP ─────────────────────────────────────────────
def setup():
    goHome()
    time.sleep(2)
    _println("hello")     # ready message

# ─────────────────── LOOP ──────────────────────────────────────────────
def loop():
    global currentMode
    global rawX_filt, rawY_filt
    global lastX, lastY
    global mode4_init

    _service_serial()

    # run mode
    if currentMode == 0:           # calibration sweep
        drawTo(-3, 29.2)
        time.sleep_ms(500)
        drawTo(74.1, 28)
        time.sleep_ms(500)

    elif currentMode == 1:         # manual pot control
        newX = adc_left.read_u16()
        newY = adc_right.read_u16()

        rawX_filt += FILTER_ALPHA * (newX - rawX_filt)
        rawY_filt += FILTER_ALPHA * (newY - rawY_filt)

        Tx = map_range(rawX_filt, 0, 65535, X_MIN, X_MAX)
        Ty = map_range(rawY_filt, 0, 65535, Y_MIN, Y_MAX)

        if abs(Tx - lastX) > DEADBAND_MM or \
           abs(Ty - lastY) > DEADBAND_MM:
            set_XY(Tx, Ty)
            lastX, lastY = Tx, Ty

    elif currentMode == 2:         # demo
        drawTo(0, 20)
        time.sleep_ms(10)
        goHome()

    elif currentMode == 3:         # constantly home
        goHome()

    elif currentMode == 4:         # command‑only
        if not mode4_init:
            goHome()
            mode4_init = True
        # then wait for commands

    time.sleep_ms(10)

# ─────────────────── MAIN ──────────────────────────────────────────────
setup()
while True:
    loop()