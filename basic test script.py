from pymavlink import mavutil

# Connect to the drone on COM3
connection = mavutil.mavlink_connection('COM3', baud=57600)  # Try 115200 if needed
print("📡 Waiting for heartbeat...")
connection.wait_heartbeat()
print("✅ Heartbeat received from system (ID {} component {})".format(
    connection.target_system, connection.target_component
))

