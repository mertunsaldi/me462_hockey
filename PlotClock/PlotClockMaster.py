import machine
import utime

uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

def send_command(cmd):
    uart.write((cmd + "\n").encode())

def read_response(timeout_ms=2000):
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < timeout_ms:
        if uart.any():
            line = uart.readline()
            if line:
                try:
                    return line.decode().strip()
                except:
                    return str(line)
        utime.sleep_ms(10)
    return None

while True:
    try:
        cmd = input()
        if cmd:
            print("Sended:", cmd)
            send_command(cmd)
            response = read_response()
            if response:
                print("Response:", response)
            else:
                print("No Responce Timeout.")
    except KeyboardInterrupt:
        print("\nExiting...")
        break
