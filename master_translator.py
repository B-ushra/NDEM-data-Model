# master_translator.py
# This is our final translator for a single drone.
# It captures multiple message types, fuses them, and generates a rich NDEM XML report.

# --- Required Libraries (Toolboxes) ---
import time
import threading
import numpy as np
from lxml import etree
from pymavlink import mavutil

# --- Configuration ---
# Use the full path to python.exe to be safe, or just 'COM3' if that works for you.
DRONE_PORT = 'COM3'  # <--- CHANGE THIS to your Hexacopter's COM port!
BAUD_RATE = 57600

# ==============================================================================
# PART 1: THE "DATA BRIDGE" - A Class to Hold Our Drone's State
# ==============================================================================
# Instead of a simple dictionary, a class is a cleaner way to organize the data.
class DroneState:
    """This class acts as our clean, organized 'data dictionary'."""
    def __init__(self):
        self.uav_id = "UAV-01-HEXA"
        self.is_landed = True
        self.gps_fix_type = "NoFix"
        self.satellites_visible = 0
        self.position_source = "Unknown"
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.north_m = 0.0
        self.east_m = 0.0
        self.down_m = 0.0
        self.battery_voltage = "Unknown"

# Create one instance of this class to hold the live state of our drone.
live_drone_state = DroneState()
stop_threads = False

# ==============================================================================
# PART 2: THE "TRANSLATOR BRAIN" - Functions to Process Raw Data
# ==============================================================================

def update_state_from_mavlink(message, state: DroneState):
    """
    This is the core translation logic. It takes a raw MAVLink message and
    updates our clean DroneState object with meaningful information.
    """
    msg_type = message.get_type()

    # --- Translate GPS Data ---
    if msg_type == 'GPS_RAW_INT':
        state.satellites_visible = message.satellites_visible
        fix_type = message.fix_type
        if fix_type >= 3:
            state.gps_fix_type = "3D_Fix"
            state.position_source = "GPS" # We have a good lock
        else:
            state.gps_fix_type = "NoFix"
            state.position_source = "InertialOnly" # Relying on internal sensors

    # --- Translate Attitude Data ---
    elif msg_type == 'ATTITUDE_QUATERNION':
        q = [message.q1, message.q2, message.q3, message.q4]
        # Using a helper function to do the math
        roll, pitch, yaw = quaternion_to_euler_degrees(q)
        state.roll_deg = roll
        state.pitch_deg = pitch
        state.yaw_deg = yaw
    
    # --- Translate Position Data ---
    elif msg_type == 'LOCAL_POSITION_NED':
        state.north_m = message.x
        state.east_m = message.y
        state.down_m = message.z

    # --- Translate System Status ---
    elif msg_type == 'EXTENDED_SYS_STATE':
        state.is_landed = (message.landed_state == 1)

    elif msg_type == 'SYS_STATUS':
        voltage = message.voltage_battery
        # 65535 is the MAVLink code for "invalid/unknown"
        state.battery_voltage = f"{voltage / 1000.0:.2f}V" if voltage != 65535 else "Unknown"

def quaternion_to_euler_degrees(q):
    """Helper function to do the quaternion math."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)) # Use clip for safety
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# ==============================================================================
# PART 3: THE "XML BUILDER" - Creates the Final NDEM Report
# ==============================================================================

def generate_ndem_xml_report(state: DroneState):
    """Takes the clean DroneState object and builds the final XML string."""
    NS_MAP = {'ndem': 'http://www.yourdomain.com/ndem'}
    root = etree.Element(f"{{{NS_MAP['ndem']}}}UAVStatusReport", nsmap=NS_MAP)

    # --- Metadata Section ---
    metadata = etree.SubElement(root, "Metadata")
    etree.SubElement(metadata, "UAVIdentifier").text = state.uav_id
    etree.SubElement(metadata, "TimestampUTC").text = time.strftime('%Y-%m-%dT%H:%M:%SZ')

    # --- Operational Status Section ---
    op_status = etree.SubElement(root, "OperationalStatus")
    etree.SubElement(op_status, "Condition").text = "Landed" if state.is_landed else "In-Air"
    etree.SubElement(op_status, "Battery").set("voltage", state.battery_voltage)

    # --- Navigation Status Section ---
    nav_status = etree.SubElement(root, "NavigationStatus")
    etree.SubElement(nav_status, "PositionSource").text = state.position_source
    gps_element = etree.SubElement(nav_status, "GPS_Status")
    gps_element.set("fixType", state.gps_fix_type)
    gps_element.set("satellitesVisible", str(state.satellites_visible))

    # --- Kinematics Section ---
    kinematics = etree.SubElement(root, "Kinematics")
    pos_element = etree.SubElement(kinematics, "Position_NED")
    pos_element.set("north_m", f"{state.north_m:.3f}")
    pos_element.set("east_m", f"{state.east_m:.3f}")
    pos_element.set("down_m", f"{state.down_m:.3f}")
    att_element = etree.SubElement(kinematics, "Attitude_Euler")
    att_element.set("roll_deg", f"{state.roll_deg:.2f}")
    att_element.set("pitch_deg", f"{state.pitch_deg:.2f}")
    att_element.set("yaw_deg", f"{state.yaw_deg:.2f}")

    # Convert the entire XML tree to a nicely formatted string for printing
    return etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8').decode('utf-8')


# ==============================================================================
# PART 4: THE "MAIN ENGINE" - Threads to Listen and Generate Reports
# ==============================================================================

def listener_thread_function(conn, state: DroneState):
    """This thread just listens for messages and calls the translator."""
    interesting_messages = [
        'GPS_RAW_INT', 
        'ATTITUDE_QUATERNION', 
        'LOCAL_POSITION_NED',
        'EXTENDED_SYS_STATE',
        'SYS_STATUS'
    ]
    while not stop_threads:
        msg = conn.recv_match(type=interesting_messages, blocking=True, timeout=1)
        if msg:
            # When a message comes in, update our central state object
            update_state_from_mavlink(msg, state)

def reporter_thread_function(state: DroneState):
    """This thread wakes up every second and generates the XML report."""
    while not stop_threads:
        # Clear the console screen
        print("\033c", end="")
        
        # Generate the XML from the current live state
        xml_report = generate_ndem_xml_report(state)
        
        # Print the report
        print(xml_report)
        
        # Wait for 1 second before generating the next report
        time.sleep(1)

# --- The Main Starting Point of the Program ---
if __name__ == "__main__":
    try:
        print("--- MASTER NDEM TRANSLATOR ---")
        print(f"Connecting to drone on {DRONE_PORT}...")
        connection = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
        connection.wait_heartbeat()
        print("Connection successful. Starting threads...")
        print("Press Ctrl+C to stop the program.")
        
        # Create and start the listener thread
        listener_thread = threading.Thread(target=listener_thread_function, args=(connection, live_drone_state))
        listener_thread.start()
        
        # Create and start the reporter thread
        reporter_thread = threading.Thread(target=reporter_thread_function, args=(live_drone_state,))
        reporter_thread.start()
        
        # Keep the main program alive while the threads do their work
        listener_thread.join()
        reporter_thread.join()

    except KeyboardInterrupt:
        print("\nStopping threads...")
        stop_threads = True
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        print("Program finished.")
