import uasyncio as asyncio
import ujson
from machine import UART, Pin

# Identifier of the slave Pico.
SLAVE_ID = "slave01"

# Define available commands and their expected parameters.
# For instance, "ACTUATE" expects a parameter like "ON" or "OFF".
available_commands = {
    "ACTUATE": ["ON", "OFF"],
    "READ_SENSOR": []  # No parameters needed for a sensor reading command.
}

# Configure the UART (adjust TX/RX pins based on your wiring)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

async def send_handshake():
    """
    Sends a handshake message to the master.
    The handshake message is a JSON string prefixed with "HANDSHAKE:".
    """
    handshake_data = {"id": SLAVE_ID, "commands": available_commands}
    message = "HANDSHAKE:" + ujson.dumps(handshake_data) + "\n"
    uart.write(message)
    print("Sent handshake:", message)
    # You might choose to send it multiple times or wait for an ACK here.
    await asyncio.sleep(1)

async def command_listener():
    """
    Continuously listens for incoming UART data containing commands.
    Expects commands in JSON format, for example:
      {"target": "slave01", "command": "ACTUATE", "params": ["ON"]}
    Only commands addressed to this slave (or a broadcast target "ALL") are processed.
    """
    while True:
        if uart.any():
            try:
                data = uart.readline()
                if data:
                    # Check if data is bytes or already a string.
                    if isinstance(data, bytes):
                        msg = data.decode('utf-8').strip()
                    else:
                        msg = data.strip()
                    
                    try:
                        # Parse the incoming JSON command.
                        cmd_data = ujson.loads(msg)
                        target = cmd_data.get("target")
                        command = cmd_data.get("command")
                        params = cmd_data.get("params", [])
                        
                        # Process only if the command is meant for this slave.
                        if target == SLAVE_ID or target == "ALL":
                            print("Received command:", command, "with params:", params)
                            await process_command(command, params)
                        else:
                            print("Message not for this slave:", target)
                    except Exception as parse_err:
                        print("Error parsing command JSON:", parse_err, "Message:", msg)
            except Exception as e:
                print("Error reading UART data:", e)
        await asyncio.sleep(0.01)

async def process_command(command, params):
    """
    Processes a command received from the master.
    This sample implementation handles ACTUATE and READ_SENSOR commands.
    Extend or modify this function based on your application.
    """
    if command == "ACTUATE":
        # Example: Turn an actuator ON or OFF based on the parameter.
        if params:
            action = params[0]
            print("Performing actuator action:", action)
            # Insert real actuator control code here.
            await asyncio.sleep(0.5)  # Simulate time taken for the action.
            response = {"source": SLAVE_ID, "response": f"Actuator {action} executed"}
            uart.write(ujson.dumps(response) + "\n")
            print("Actuation response sent:", response)
        else:
            print("ACTUATE command received with no parameters")
    elif command == "READ_SENSOR":
        # Simulate a sensor reading.
        sensor_value = 42  # Replace with actual sensor read logic.
        print("Sensor reading:", sensor_value)
        response = {"source": SLAVE_ID, "response": f"Sensor value: {sensor_value}"}
        uart.write(ujson.dumps(response) + "\n")
        print("Sensor response sent:", response)
    else:
        print("Unknown command:", command)
        response = {"source": SLAVE_ID, "response": "Unknown command"}
        uart.write(ujson.dumps(response) + "\n")
        print("Unknown command response sent.")

async def sensor_task():
    """
    Periodically sends sensor data as an outgoing message.
    This task runs independently of the command listener and simulates periodic sensor updates.
    """
    while True:
        # Simulate reading a sensor value.
        sensor_value = 42  # Replace with your sensor read code.
        message = {"source": SLAVE_ID, "sensor": sensor_value}
        uart.write(ujson.dumps(message) + "\n")
        print("Sent sensor data:", message)
        await asyncio.sleep(10)  # Adjust frequency as needed.

async def main():
    # Send the handshake to let the master know this slave is online.
    await send_handshake()
    
    # Create tasks for listening to commands and for sending sensor data.
    asyncio.create_task(command_listener())
    asyncio.create_task(sensor_task())
    
    # The main loop can be used for additional tasks or a heartbeat.
    while True:
        # Optionally, add a heartbeat or other periodic tasks here.
        await asyncio.sleep(1)

try:
    asyncio.run(main())
except Exception as e:
    print("Main loop error:", e)