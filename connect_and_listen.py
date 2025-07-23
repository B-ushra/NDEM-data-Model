
# connect_and_listen.py
# Our first program! Its only job is to connect to one drone and listen.

from pymavlink import mavutil
import time

# --- IMPORTANT CONFIGURATION ---
# Change this to the COM port of your Hexacopter's telemetry radio.
# On Windows, it looks like 'COM3', 'COM4', etc.
# On Linux or Mac, it looks like '/dev/ttyUSB0' or '/dev/tty.usbmodem1'.
DRONE_PORT = 'COM3'  # <--- CHANGE THIS!
BAUD_RATE = 57600   # This is usually 57600 for SiK radios

print(f"Attempting to connect to drone on {DRONE_PORT} at {BAUD_RATE} baud...")

try:
    # This is the line that tries to make the connection.
    drone_connection = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)

    # We need to wait for the drone to send a "heartbeat" message to confirm it's alive.
    drone_connection.wait_heartbeat()
    print("Heartbeat received! Drone is connected.")
    print("---------------------------------------")
    print("Now listening for messages. Press Ctrl+C to stop.")
    
    # This is an endless loop to keep listening.
    while True:
        # Wait for any message to arrive.
        message = drone_connection.recv_match(blocking=True)
        
        # As soon as a message arrives, print it to the screen.
        print(message)
        
except Exception as e:
    print(f"Failed to connect or an error occurred: {e}")
    print("Please check the COM port, make sure the drone is powered on, and the radio is connected.")

except KeyboardInterrupt:
    print("\nProgram stopped by user.")