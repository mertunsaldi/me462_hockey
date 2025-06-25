import machine
import utime
import select
import sys

uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

# ─────────────────── USB‑SERIAL (non‑blocking) ─────────────────────────
_serial_poll = select.poll()
_serial_poll.register(sys.stdin, select.POLLIN)
_serial_buf = bytearray()

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
print("Command: ")
while True:
    try:
        cmd = _serial_buf
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
