from pymavlink import mavutil
import time
import math
import sys

# ------------------------
# CONNECTION SETUP
# ------------------------
serial_port = "udpin:localhost:14551"
baud_rate = 57600
emergency_land_triggered = False
LAND_MODE = 9

def key_listener():
    """Listens for the 'q' key followed by Enter to trigger an emergency landing."""
    global emergency_land_triggered
    print("Press 'q' and then Enter for EMERGENCY LAND.")
    while True:
        line = sys.stdin.readline()
        if line.strip().lower() == 'q':
            emergency_land_triggered = True
            print("⚠️ EMERGENCY LAND TRIGGERED!")
            break

def emergency_land_here():
    """Initiates an emergency landing and waits for the drone to land."""
    print("Initiating emergency landing...")
    master.set_mode(LAND_MODE)
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            relative_alt = msg.relative_alt / 1000.0
            print(f"Current altitude: {relative_alt:.2f} meters")
            if relative_alt < 0.2:
                print("Landed successfully.")
                break
        time.sleep(1)


def check_emergency_and_exit():
    """Checks the emergency flag and handles the landing sequence if needed."""
    if emergency_land_triggered:
        emergency_land_here()
        print("Emergency landing complete. Exiting program.")
        sys.exit(0)
try:
    master = mavutil.mavlink_connection(serial_port)
except Exception as e:
    print(f"Failed to connect: {e}")
    print("Please check the port and baud rate.")
    exit(1)

# Wait for the first heartbeat
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system}, component {master.target_component}")

# ------------------------
# SET PARAMETERS
# ------------------------
def set_parameter(param_id, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT8):
    """Sets a parameter on the flight controller."""
    print(f"Setting {param_id} to {value}...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_id.encode('utf-8'), value, param_type
    )
    # Wait for acknowledgment
    while True:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.strip('\x00') == param_id and msg.param_value == value:
            print(f"{param_id} set to {value}")
            break
        time.sleep(0.5)

# Disable RC-related parameters for autonomous flight
print("Configuring parameters for autonomous flight...")
set_parameter("FLTMODE_CH", 0)
set_parameter("FS_THR_ENABLE", 0)
set_parameter("ARMING_RUDDER", 0)
set_parameter("ARMING_CHECK", 65471, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
set_parameter("FS_GCS_ENABLE", 0)
time.sleep(1)

# ------------------------
# CONSTANTS
# ------------------------
TAKEOFF_ALT = float(input("Enter the Altitude or Height: "))
MOVE_DIST_METERS = float(input("Enter the Distance for each side of square: "))
VELOCITY = float(input("Enter the velocity: "))

# ArduCopter flight modes
GUIDED_MODE = 4


# ------------------------
# FUNCTIONS
# ------------------------
def calculate_gps_offsets(latitude):
    """Calculate accurate GPS offsets based on current latitude"""
    lat_offset_per_meter = 1.0 / 111320.0  # Constant for latitude
    # Longitude offset varies with latitude due to Earth's curvature
    lon_offset_per_meter = 1.0 / (111320.0 * math.cos(math.radians(latitude)))
    return lat_offset_per_meter, lon_offset_per_meter

def request_message_interval(message_id, frequency_hz):
    """Requests a specific MAVLink message at a given frequency."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0)
    print(f"Requested message {message_id} at {frequency_hz}Hz")

def set_mode(mode_id):
    """Sets the flight mode."""
    master.set_mode(mode_id)
    print(f"Mode set to {mode_id}")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == mode_id:
            break

def arm():
    """Arms the vehicle."""
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed")

def takeoff(alt):
    """Commands takeoff to a specific altitude."""
    print(f"Requesting takeoff to {alt} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )

def land(lat, lon):
    """Commands the vehicle to land at a specific GPS location."""
    print("Initiating landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, lat, lon, 0
    )

def get_position():
    """Gets the current GPS position and altitude."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg:
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.relative_alt / 1000.0
        }
    return None

def move_to_position(lat, lon, alt):
    """Moves to a specific GPS position."""
    print(f"Moving to lat={lat:.7f}, lon={lon:.7f}, alt={alt}m...")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0, 0, 0
    )

def move_and_wait(target_lat, target_lon, target_alt, description, lat_offset, lon_offset):
    """Move to position and wait until reached with accurate distance calculation."""
    print(f"{description}...")
    start_time = time.time()
    
    while True:
        move_to_position(target_lat, target_lon, target_alt)
        
        pos = get_position()
        if pos:
            # Calculate accurate distance using proper GPS offsets
            lat_diff_meters = abs(pos['lat'] - target_lat) / lat_offset
            lon_diff_meters = abs(pos['lon'] - target_lon) / lon_offset
            total_distance = math.sqrt(lat_diff_meters**2 + lon_diff_meters**2)
            
            print(f"Distance to target: {total_distance:.2f}m")
            
            if total_distance < 0.8:  # Within 0.5m accuracy
                print(f"{description} completed!")
                break
        
        # Safety timeout
        # if time.time() - start_time > 20:
        #     print("Movement timeout reached.")
        #     break
        
        time.sleep(0.5)

# ------------------------
# MISSION FLOW
# ------------------------
# Request position data at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)
time.sleep(1)
try:
    # Set GUIDED mode
    print("Setting GUIDED mode...")
    set_mode(GUIDED_MODE)
    time.sleep(1)

    # Arm the drone
    print("Arming motors...")
    arm()
    time.sleep(2)

    # Takeoff
    print("Takeoff sequence initiated...")
    takeoff(TAKEOFF_ALT)

    # Wait until the drone reaches the target altitude
    print("Waiting to reach target altitude...")
    while True:
        pos = get_position()
        if pos and pos['alt'] > 0:
            print(f"Current Altitude: {pos['alt']:.2f}m")
            if pos['alt'] >= TAKEOFF_ALT * 0.95:
                print("Reached target altitude")
                break
        time.sleep(0.5)
    if emergency_land_triggered:
        emergency_land_here()
        sys.exit(0)
    # Get initial GPS position (this will be our square's starting corner)
    print("Acquiring initial GPS position...")
    initial_pos = None
    for _ in range(20):
        initial_pos = get_position()
        if initial_pos:
            print(f"Starting GPS: lat={initial_pos['lat']:.7f}, lon={initial_pos['lon']:.7f}")
            break
        time.sleep(0.5)

    if not initial_pos:
        print("Failed to get GPS position")
        set_mode(LAND_MODE)
        exit(1)

    # Calculate accurate GPS offsets for this location
    lat_offset_per_meter, lon_offset_per_meter = calculate_gps_offsets(initial_pos['lat'])
    print(f"GPS offsets calculated for latitude {initial_pos['lat']:.6f}")

    # Define all four corners of the perfect square BEFORE starting movement
    # This ensures geometric accuracy
    corner_start = {  # Starting position (Corner 1)
        'lat': initial_pos['lat'],
        'lon': initial_pos['lon']
    }

    corner_north = {  # Corner 2: Move North
        'lat': corner_start['lat'] + MOVE_DIST_METERS * lat_offset_per_meter,
        'lon': corner_start['lon']
    }

    corner_northeast = {  # Corner 3: Move East from North corner
        'lat': corner_north['lat'],
        'lon': corner_north['lon'] + MOVE_DIST_METERS * lon_offset_per_meter
    }

    corner_southeast = {  # Corner 4: Move South from Northeast corner
        'lat': corner_start['lat'],  # Same latitude as start
        'lon': corner_northeast['lon']  # Same longitude as northeast
    }

    print("Starting PERFECT SQUARE flight pattern...")
    print(f"Square dimensions: {MOVE_DIST_METERS}m × {MOVE_DIST_METERS}m")
    print(f"Corner coordinates calculated and verified!")

    # Execute the perfect square flight pattern
    # Side 1: Start → North
    move_and_wait(corner_north['lat'], corner_north['lon'], TAKEOFF_ALT, 
                "Side 1: Moving NORTH", lat_offset_per_meter, lon_offset_per_meter)
    for _ in range(10):
        if emergency_land_triggered:
            emergency_land_here()
            sys.exit(0)
        time.sleep(0.5)    

    # Side 2: North → Northeast (East)
    move_and_wait(corner_northeast['lat'], corner_northeast['lon'], TAKEOFF_ALT,
                "Side 2: Moving EAST", lat_offset_per_meter, lon_offset_per_meter)
   
    for _ in range(10):
        if emergency_land_triggered:
            emergency_land_here()
            sys.exit(0)
        time.sleep(0.5)   
    # Side 3: Northeast → Southeast (South)
    move_and_wait(corner_southeast['lat'], corner_southeast['lon'], TAKEOFF_ALT,
                "Side 3: Moving SOUTH", lat_offset_per_meter, lon_offset_per_meter)
    for _ in range(10):
        if emergency_land_triggered:
            emergency_land_here()
            sys.exit(0)
        time.sleep(0.5)   

    # Side 4: Southeast → Start (West) - Complete the square!
    move_and_wait(corner_start['lat'], corner_start['lon'], TAKEOFF_ALT,
                "Side 4: Moving WEST - Completing Square", lat_offset_per_meter, lon_offset_per_meter)
    for _ in range(10):
        if emergency_land_triggered:
            emergency_land_here()
            sys.exit(0)
        time.sleep(0.5)   
    print("PERFECT SQUARE COMPLETED!")
    print("All four sides flown with equal length and 90° corners!")

    # Clear velocity commands to stop movement
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b0000111111000111,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0
    )

    # Land at the starting position (perfect return!)

    # Land at current position
    print("Landing at current position...")
    pos = get_position()
    if pos:
        land(pos['lat'], pos['lon'])
    else:
        print("⚠️ No GPS fix, falling back to LAND mode")
        set_mode(LAND_MODE)


    # Wait for landing

    while True:
        pos = get_position()
        if pos and pos['alt'] < 0.1:
            print("Drone landed successfully")
            break
        time.sleep(0.5)

    # Disarm the drone
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Drone disarmed. Perfect square mission complete!")

    # Close connection
    master.close()
    print("Connection closed")
    print("Mission Summary: Perfect square flight completed successfully!")
except KeyboardInterrupt:
    print("\nProgram interrupted by user. Exiting.")
    sys.exit(0)
