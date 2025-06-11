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
SERVO_LEFT_FACTOR  = 600
SERVO_RIGHT_FACTOR = 600

SERVO_LEFT_NULL  = 560
SERVO_RIGHT_NULL = 480

L1 = 210   # Servo arm length
L4 = 280   # Tip-point arm length
L5 = 150   # Distance between the two servos

MOVE_DELAY_MS = 0

O1X, O1Y = -L5/2, 0  # Left servo origin (midpoint chosen as center between servos)
O2X, O2Y = L5/2, 0   # Right servo origin

beta = math.acos(L5 / (2 * L4))
HOME_X, HOME_Y = 0, L1 + L4 * math.sin(beta)  # Home position (perpendicular servo arms)

# ─────────────────── CALIBRATION POINTS ───────────────────────────────
alpha = math.acos(L5 / (2 * L1))
a = 1
b = -2 * L1 * math.cos(math.pi/4)
c = 2 * L1 - 2 * L4
delta = b**2 - 4 * a * c

L6 = (-b + math.sqrt(delta)) / (2 * a)
CalibrationPoint2x = L5/2 + L6 * math.cos(math.pi/2 - alpha)
CalibrationPoint1x = -CalibrationPoint2x
CalibrationPointy = L6 * math.sin(math.pi/2 - alpha)

# ─────────────────── GLOBALS ───────────────────────────────────────────
currentMode = 3       # 0=cal,1=manual,2=demo,3=home‑loop,4=command‑only
mode4_init  = False
lastX, lastY = HOME_X, HOME_Y

# ─────────────────── HARDWARE OBJECTS ──────────────────────────────────
servo_left  = Servo(SERVO_LEFT_PIN)
servo_right = Servo(SERVO_RIGHT_PIN)

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

# ─────────────────── Area Check ─────────────────────────────────────────────          
def checkArea(Tx,Ty):
    dx1 = Tx - O1X
    dy1 = Ty - O1Y
    c1 = math.sqrt(dx1*dx1 + dy1*dy1)

    dx2 = Tx - O2X
    dy2 = Ty - O2Y
    c2 = math.sqrt(dx2*dx2 + dy2*dy2)

    if c1 > L1+ L4 or c2 > L1 + L4:
        print("Point (",Tx,Ty,") is outside of the reachable area")
        return False
    return True

# ─────────────────── KINEMATICS ────────────────────────────────────────
def set_XY(Tx, Ty):
    # Calculate angle for left servo arm

    dx = Tx - O1X
    dy = Ty - O1Y
    c = math.sqrt(dx*dx + dy*dy)
    a1 = math.atan2(dy, dx)
    a2 = return_angle(L1, L4, c)  # Both arms connected to servo have length L1 now
     
    print("Left ",a1-a2)
    servo_left.writeMicroseconds(
        int(((-math.pi/4 + (a1 + a2)) * SERVO_LEFT_FACTOR) + SERVO_LEFT_NULL)
    )

    # Calculate angle for right servo arm
    dx2 = Tx - O2X
    dy2 = Ty - O2Y
    c2 = math.sqrt(dx2*dx2 + dy2*dy2)
    a1b = math.atan2(dy2, dx2)
    a2c = return_angle(L1, L4, c2)  # Right arm link lengths are L1 and L4
    
    print("Comparison ", a2,a2c)
    print("Right ",a1b + a2c)
    servo_right.writeMicroseconds(
        int(((math.pi/4 + (a1b - a2c)) * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL)
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
    time.sleep(0.005)

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
            # ——— update current position for the next drawTo() ———
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
    global lastX, lastY
    global mode4_init

    _service_serial()

    if currentMode == 0:           # calibration sweep
        drawTo(CalibrationPoint1x, CalibrationPointy)
        time.sleep_ms(500)
        drawTo(CalibrationPoint2x, CalibrationPointy)
        time.sleep_ms(500)

    elif currentMode == 1:         
        servo_left.writeMicroseconds(SERVO_LEFT_NULL)
        servo_right.writeMicroseconds(SERVO_RIGHT_NULL)
        time.sleep(2)
        servo_left.writeMicroseconds(SERVO_LEFT_NULL + SERVO_LEFT_FACTOR * math.pi / 2)
        servo_right.writeMicroseconds(SERVO_RIGHT_NULL + SERVO_RIGHT_FACTOR * math.pi / 2)
        time.sleep(2)
        servo_left.writeMicroseconds(SERVO_LEFT_NULL + SERVO_LEFT_FACTOR * math.pi)
        servo_right.writeMicroseconds(SERVO_RIGHT_NULL + SERVO_RIGHT_FACTOR * math.pi)
        time.sleep(2)

    elif currentMode == 2:         # demo
        servo_left.writeMicroseconds(SERVO_LEFT_NULL+SERVO_LEFT_FACTOR*math.pi/4)
        servo_right.writeMicroseconds(SERVO_RIGHT_NULL+SERVO_RIGHT_FACTOR*3*math.pi/4)

    elif currentMode == 3:         # constantly home
        goHome()

    elif currentMode == 4:         # command‑only
        if not mode4_init:
            goHome()
            mode4_init = True
        # wait for commands

    time.sleep(0.005)

# ─────────────────── MAIN ──────────────────────────────────────────────
if __name__ == "__main__":
    #setup()
    print(checkArea(0,500))
    while True:
        #loop()
        pass

