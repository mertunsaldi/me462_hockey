import machine
import math
import utime



utime.sleep_ms(1000	)

DEVICE_ID = "P0"

class Servo:
    def __init__(self, pin, factor = 1000, startPos = 1000):
        self.pwm = machine.PWM(machine.Pin(pin))
        self.pwm.freq(50)
        self.angle = 0 # radian
        self.factor = factor #servoFactor choosen close to 1000 and calibrate
        self.startPos = startPos #startPos choosen close to 1000 and calibrate

    def getAngle(self):
        return self.angle
    
    def writeMicroseconds(self, us):
        self.pwm.duty_u16(int(us / 20000 * 65535))

    def setAngle(self, rad):
        us = self.startPos - 500 + 2* (rad * self.factor / math.pi)
        self.writeMicroseconds(us)
        self.angle = rad

    def calibrateRotation(self,factor): 
        self.factor = factor
        self.setAngle(0)
        wait_ms(2000)
        self.setAngle(math.pi/2)
        wait_ms(2000)
        self.setAngle(math.pi)
        wait_ms(2000)

class PlotClock:
    def __init__(self,servoLeft,servoRight,L1,L2,L3,L4,hitDia,servoMarginAngle = 2.5/180*math.pi):
        self.servoLeft = servoLeft
        self.servoRight = servoRight
        self.L1 = L1    # L1: ServoLink length
        self.L2 = L2    # L2: TipLink length
        self.L3 = L3    # L3: Distance between Servos
        self.L4 = L4    # L4: Distance from servo to wall front face
        self.hitDia = hitDia # HitterDiameter
        self.servoMarginAngle = servoMarginAngle
        self.posX = 0
        self.posY = 0
        self._move_active = False
        self._last_tick = utime.ticks_ms()
        self._stepX = 0
        self._stepY = 0
        self._n_steps = 0
        self._step_index = 0
        self._stepDelay = 20

        self.gripper_servo = None

        self.initialize()
    
    def setup(self,L1,L2,L3,L4,hitDia,servoMarginAngle = 2.5/180*math.pi):
        self.L1 = L1    # L1: ServoLink length
        self.L2 = L2    # L2: TipLink length
        self.L3 = L3    # L3: Distance between Servos
        self.L4 = L4    # L4: Distance from servo to wall front face
        self.hitDia = hitDia # HitterDiameter
        self.servoMarginAngle = servoMarginAngle
        self.initialize()

    def initialize(self):
        self.O1X, self.O1Y = -self.L3/2, 0  # Left servo position (midpoint chosen as center between servos)
        self.O2X, self.O2Y = self.L3/2, 0   # Right servo position       

        beta = math.acos(self.L3 / (2 * self.L2))
        self.HOME_X, self.HOME_Y = 0, self.L1 + self.L2 * math.sin(beta)  # Home position (perpendicular servo arms)
        self.startAngle = math.asin((self.L2-self.hitDia/2-self.L4)/self.L1) + self.servoMarginAngle
        gamma = math.asin((self.L1*math.sin(self.startAngle+self.servoMarginAngle)-self.L4-self.hitDia/2)/self.L2)
        self.max_x_distance =  self.O1X + self.L1 * math.cos(self.startAngle + self.servoMarginAngle) + self.L2 * math.cos(gamma)
        self.min_y_distance = self.O1Y + self.L4 + self.hitDia/2

    def getXY (self):
        return (self.posX, self.posY)
    
    def getMaxX (self):
        return self.max_x_distance
    
    def getMinY (self):
        return  self.min_y_distance

    def getStartAngle(self):
        return self.startAngle

    def getL1(self):
        return self.L1

    def getL2(self):
        return self.L2

    def getL3(self):
        return self.L3

    def getL4(self):
        return self.L4
    
    def startPoseCalibration(self,leftStartPose,rightStartPose):
        self.servoLeft.startPose = leftStartPose
        self.servoRight.startPose = rightStartPose
        self.goHome()

    def goHome(self):

        self.gotoXY(self.HOME_X,self.HOME_Y)

    def checkTarget(self, Tx, Ty):
        dx1 = Tx - self.O1X
        dy1 = Ty - self.O1Y
        c1 = math.sqrt(dx1*dx1 + dy1*dy1)

        dx2 = Tx - self.O2X
        dy2 = Ty - self.O2Y
        c2 = math.sqrt(dx2*dx2 + dy2*dy2)

        if (c1 > self.L1 + self.L2 or c2 > self.L1 + self.L2 or abs(Tx) > self.max_x_distance
        or c1 < abs(self.L1 - self.L2) or c2 < abs(self.L1 - self.L2) or Ty < self.min_y_distance):
            #print("Point (",Tx,Ty,") is outside of the reachable area")
            return False
        return True
    
    def gotoXY(self, Tx, Ty):
    # Set Tip Point
        if self.checkTarget(Tx, Ty):
            # Calculate angle for left servo arm
            dx1 = Tx - self.O1X
            dy1 = Ty - self.O1Y
            c1 = math.sqrt(dx1*dx1 + dy1*dy1)
            a11 = math.atan2(dy1, dx1)
            a12 = cosTheomAng(self.L1, c1, self.L2) 

            # Calculate angle for right servo arm
            dx2 = Tx - self.O2X
            dy2 = Ty - self.O2Y
            c2 = math.sqrt(dx2*dx2 + dy2*dy2)
            a21 = math.atan2(dy2, dx2)
            a22 = cosTheomAng(self.L1, c2, self.L2) 

            self.servoLeft.setAngle(a11 + a12 - self.startAngle)
            self.posX = Tx
            self.servoRight.setAngle(a21 - a22 + self.startAngle)
            self.posY = Ty

    def setXY(self, Tx, Ty, n_steps=100, stepDelay=2):
        self._stepDelay = stepDelay
        self._step_index = 0
        self._stepX = (Tx-self.posX)/n_steps
        self._stepY = (Ty-self.posY)/n_steps
        self._n_steps = n_steps

    def setXYrel(self, Tx, Ty, n_steps=100, stepDelay=2):
        self._stepDelay = stepDelay
        self._step_index = 0
        self._stepX = (Tx)/n_steps
        self._stepY = (Ty)/n_steps
        self._n_steps = n_steps

        
    def move(self):
        now = utime.ticks_ms()
        if utime.ticks_diff(now, self._last_tick) >= self._stepDelay and self._step_index < self._n_steps:
            self._step_index += 1
            destX = self.posX + self._stepX
            destY = self.posY + self._stepY
            self.gotoXY(destX, destY)
            self._last_tick = now

    def update(self):
        self.move()

    def _set_gripper_angle(self, angle):
        if self.gripper_servo is None:
            return
        angle = max(0, min(180, angle))
        min_duty = 1638
        max_duty = 8192
        duty = int(min_duty + (max_duty - min_duty) * angle / 180)
        self.gripper_servo.duty_u16(duty)

    def grip(self):
        self._set_gripper_angle(50)

    def release(self):
        self._set_gripper_angle(180)

    def grip_smooth(self, end_angle=50, step=5, delay=0.01):
        if self.gripper_servo is None:
            return
        for angle in range(180, end_angle - 1, -step):
            self._set_gripper_angle(angle)
            utime.sleep(delay)

    def release_smooth(self, end_angle=180, step=2, delay=0.01):
        if self.gripper_servo is None:
            return
        for angle in range(50, end_angle + 1, step):
            self._set_gripper_angle(angle)
            utime.sleep(delay)
               
    
def cosTheomAng(adjacentSide1, adjacentSide2, oppositeSide):
     return math.acos(max(-1.0, min(1.0, (adjacentSide1**2 + adjacentSide2**2 - oppositeSide**2) / (2.0 * adjacentSide1 * adjacentSide2))))

def wait_ms(ms):
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < ms:
        pass




uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))


#servoLeft = Servo(4,975,1000)
#servoRight = Servo(3,1000,1025)
#plotClock = PlotClock(servoLeft, servoRight,65,95,25,30,20)

servoLeft = Servo(4,980,1030)
servoRight = Servo(3,960,915)
plotClock = PlotClock(servoLeft, servoRight, 267,328,148,98,20)

gripper_servo = machine.PWM(machine.Pin(5))
gripper_servo.freq(50)
plotClock.gripper_servo = gripper_servo
plotClock.release()

device_objects = {
    'p': plotClock,
    'sl': servoLeft,
    'sr': servoRight,
}
def parse_command(cmd):
    # Komut örneği: "P0.p.goHome()" veya "P0.sl.setAngle(1.57)"
    id_part, sep, code_part = cmd.partition(".")
    if id_part != DEVICE_ID:
        return
    try:
        obj_name, sep2, method_part = code_part.partition(".")
        if obj_name in device_objects:
            result = eval(f'device_objects["{obj_name}"].{method_part}')
            if result is not None:
                uart.write(f"{DEVICE_ID}:{str(result)}\n".encode())  # Güvenli encode
        else:
            uart.write(f"{DEVICE_ID}:ERR:Unknown object \"{obj_name}\"\n".encode())
    except Exception as e:
        uart.write(f"{DEVICE_ID}:ERR:{e}\n".encode())

buffer = b""

def readCommand(buffer):
    try:
        if uart.any():
            data = uart.read(uart.any())
            if data:
                buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                try:
                    cmd = line.decode().strip()
                    parse_command(cmd)
                except UnicodeError:
                    # Geçersiz karakter varsa komutu atla
                    pass
    except Exception as e:
        uart.write(f"{DEVICE_ID}:ERR:UART exception {e}\n".encode())
    return buffer
led = machine.Pin("LED", machine.Pin.OUT)  # Dahili LED
led_on = False
interval = 500  # milisaniye
last_toggle_time = utime.ticks_ms()

# Move to a convenient idle location after boot
plotClock.gotoXY(0, 250)

uart.write(f"{DEVICE_ID}:READY\n".encode())  # Cihaz hazır mesajı


while True:
    buffer = readCommand(buffer)
    plotClock.update()
    
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_toggle_time) >= interval:
        led_on = not led_on
        led.value(led_on)
        last_toggle_time = current_time



