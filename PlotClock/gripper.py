import machine
import utime

# GP2 pinine bağlı servo tanımı
servo = machine.PWM(machine.Pin(2))
servo.freq(50)

# Açıdan duty cycle'a çevirme (0-180 derece)
def set_angle(angle):
    # Duty değerini hesapla (0.5ms - 2.5ms arası)
    min_duty = 1638   # 0.5ms duty (65535 * 0.5ms / 20ms)
    max_duty = 8192   # 2.5ms duty (65535 * 2.5ms / 20ms)
    duty = int(min_duty + (max_duty - min_duty) * angle / 180)
    servo.duty_u16(duty)

# Smooth geçiş fonksiyonu
def move_smooth(start_angle, end_angle, step=1, delay=0.02):
    if start_angle < end_angle:
        for angle in range(start_angle, end_angle + 1, step):
            set_angle(angle)
            utime.sleep(delay)
    else:
        for angle in range(start_angle, end_angle - 1, -step):
            set_angle(angle)
            utime.sleep(delay)

set_angle(180)
utime.sleep(3)
# Örnek kullanım
while True:
    move_smooth(180, 50)   # 180'den 0 dereceye yavaşça geri dön
    utime.sleep(2)
    move_smooth(50, 180)   # 0'dan 180 dereceye yavaşça git
    utime.sleep(2)