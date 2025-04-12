import uasyncio as asyncio
import ujson
from machine import UART, Pin

# Global dictionary to store info about the connected slaves.
# Key: slave id/name; Value: dictionary of commands and parameters.
slaves = {}

# Configure the UART (adjust TX/RX pins as needed for your wiring)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

async def handshake_listener():
    """
    Continuously checks the UART for incoming messages.
    Expects handshake messages starting with "HANDSHAKE:" where the rest is a JSON string.
    Parses the message and updates the 'slaves' dictionary.
    """
    print("Starting handshake listener...")
    while True:
        if uart.any():
            try:
                data = uart.readline()
                if data:
                    # Check if we need to decode (if it's bytes, decode; if it's already a str, use it as is)
                    if isinstance(data, bytes):
                        message = data.decode('utf-8').strip()
                    else:
                        message = data.strip()
                    if message.startswith("HANDSHAKE:"):
                        json_str = message[len("HANDSHAKE:"):].strip()
                        try:
                            handshake_data = ujson.loads(json_str)
                            slave_id = handshake_data.get("id")
                            commands = handshake_data.get("commands")
                            if slave_id and commands:
                                slaves[slave_id] = commands
                                print("Registered slave '{}' with commands: {}".format(slave_id, commands))
                            else:
                                print("Incomplete handshake data:", handshake_data)
                        except Exception as json_err:
                            print("Error parsing handshake JSON:", json_err)
                    else:
                        print("Received non-handshake message:", message)
            except Exception as e:
                print("Error processing incoming UART data:", e)
        await asyncio.sleep(0.01)

async def periodic_report():
    """Periodically prints out the registered slave dictionary."""
    while True:
        await asyncio.sleep(5)
        print("Current registered slaves:", slaves)

async def main():
    asyncio.create_task(handshake_listener())
    asyncio.create_task(periodic_report())
    while True:
        await asyncio.sleep(1)

try:
    asyncio.run(main())
except Exception as e:
    print("Main loop error:", e)