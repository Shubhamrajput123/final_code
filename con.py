import time
import sys
import threading
from pymavlink import mavutil

# --- CONSTANTS ---

# Connection details
# NOTE: This should match the output port of MAVProxy/SITL (usually 14551 or 14550)
CONNECTION_STRING = 'udpin:localhost:14551'
TARGET_ALTITUDE_METERS = 2.0  # Initial takeoff altitude
CRUISE_SPEED_MPS = 1.5        # Horizontal movement speed (meters/second)
VERTICAL_SPEED_MPS = 0.5      # Vertical movement speed (meters/second)
SEND_RATE_HZ = 5.0            # Rate at which to send continuous velocity commands

# MAVLink Frame and Mask Definitions for SET_POSITION_TARGET_LOCAL_NED (Message 84)
# MAV_FRAME_BODY_NED (8): Velocity is relative to the DRONE'S HEAD/Body
MAV_FRAME_BODY_NED_NUM = 8

# This mask ignores everything *except* VX, VY, and VZ (velocity components)
# Binary: 0b0000111111000111 = 4039 (Dec)
MASK_SET_VELOCITY_ONLY = 0b0000111111000111 

# --- GLOBAL STATE ---
# Current desired velocities (used by the background sender thread)
desired_vx = 0.0
desired_vy = 0.0
desired_vz = 0.0

# Synchronization lock for changing velocity state
state_lock = threading.Lock()

# --- MAVLINK FUNCTIONS ---

def get_current_altitude(master):
    """Returns the current relative altitude as a formatted string."""
    alt = get_raw_relative_altitude(master)
    return f"{alt:.2f}m" if alt is not None else "N/A"

def get_raw_relative_altitude(master):
    """Returns the current relative altitude in meters as a float, or None."""
    # Attempt to read a VFR_HUD message
    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg:
        # alt is relative altitude above home/takeoff point (in meters)
        return msg.alt
    
    # Fallback to GLOBAL_POSITION_INT
    msg_global = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg_global:
        # relative_alt is in millimeters, convert to meters
        return msg_global.relative_alt / 1000.0

    return None

def wait_for_heartbeat(master):
    """Wait for a heartbeat packet from the autopilot."""
    print(f"Connecting to MAVLink endpoint: {CONNECTION_STRING}")
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}.")

def guided_velocity_takeoff(master, altitude):
    """Arms motors and sends continuous vertical velocity until target altitude is reached."""
    
    # Command constant UP velocity (Negative VZ in NED frame)
    climb_vx, climb_vy = 0.0, 0.0
    climb_vz = -VERTICAL_SPEED_MPS 
    
    # 1. Check and set Guided Mode
    if master.flightmode != 'GUIDED':
        print("Changing to GUIDED mode...")
        master.set_mode('GUIDED') 
        
        start_wait = time.time()
        # Wait until the mode change is confirmed, or timeout (5 seconds)
        while master.flightmode != 'GUIDED' and (time.time() - start_wait) < 5:
            # Force the master object to process any buffered messages (like HEARTBEAT)
            master.recv_match(type=None, blocking=False)
            time.sleep(0.1) 
        
        if master.flightmode != 'GUIDED':
            print("[ERROR] Failed to confirm GUIDED mode change after 5 seconds. Aborting takeoff.")
            return False
            
    print("Vehicle is in GUIDED mode.")

    # --- NEW: Flush Stale Altitude Readings Before Arming ---
    print("Pre-flight check: Flushing stale MAVLink messages...")
    start_wait = time.time()
    # Read messages until we get a value close to zero or time out (5s)
    while (time.time() - start_wait) < 5: 
        current_alt = get_raw_relative_altitude(master)
        master.recv_match(type=None, blocking=False) # Keep the buffer moving
        
        if current_alt is not None:
            print(f"Current Alt Reading: {current_alt:.2f}m", end='\r')
            # Assuming a good reading is < 1 meter while on the ground
            if current_alt < 1.0: 
                print(f"Altitude reading stabilized near ground ({current_alt:.2f}m).")
                break
        
        time.sleep(0.1)
    else:
        # If the loop finished without breaking, it means we timed out or the reading is stuck high
        if current_alt is not None and current_alt > 1.0:
            print(f"\n[WARNING] Altitude reading stuck at {current_alt:.2f}m. Continuing, but takeoff may fail.")
    # --------------------------------------------------------

    # 2. Arming motors
    print("Arming motors...")
    master.arducopter_arm()
    
    # 3. Immediately send the vertical velocity command 
    send_velocity_command(master, climb_vx, climb_vy, climb_vz)
    print(f"Initiating vertical climb to {altitude} meters using velocity commands.")
    
    # We wait briefly for arming confirmation, though the command was already sent.
    master.motors_armed_wait()
    print("Motors armed successfully!")
    
    # 4. Velocity-based Takeoff Sequence Loop
    start_time = time.time()
    MAX_TAKEOFF_WAIT = 15 # seconds
    
    print("Waiting for drone to reach target altitude...")

    while time.time() - start_time < MAX_TAKEOFF_WAIT:
        current_alt = get_raw_relative_altitude(master)
        
        # Continuously send the vertical climb command (redundant but safe)
        send_velocity_command(master, climb_vx, climb_vy, climb_vz)
        
        # Check for unexpected disarm
        if not master.motors_armed():
            print("\n[ALERT] Takeoff aborted: Drone unexpectedly disarmed!")
            return False

        if current_alt is not None:
            print(f"Current Alt: {current_alt:.2f}m / Target Alt: {altitude}m", end='\r')
            
            # Use a tolerance (e.g., 95% of target altitude)
            if current_alt > altitude * 0.95:
                # 5. Success: Immediately send zero velocity to stop the climb and hover
                send_velocity_command(master, 0.0, 0.0, 0.0) 
                print(f"\nAltitude reached. Vehicle stabilized at {current_alt:.2f}m. Ready for movement.")
                return True
        
        # Sleep briefly to avoid overwhelming the connection
        time.sleep(0.2) 

    # 6. Timeout
    # Ensure drone stops if timeout occurs
    send_velocity_command(master, 0.0, 0.0, 0.0)
    print(f"\n[WARNING] Takeoff timeout after {MAX_TAKEOFF_WAIT}s. Hovering at current height.")
    return False

def send_velocity_command(master, vx, vy, vz):
    """
    Send MAVLink SET_POSITION_TARGET_LOCAL_NED message to command velocity.

    vx: Velocity in X (Forward/Back) in m/s
    vy: Velocity in Y (Right/Left) in m/s
    vz: Velocity in Z (Down/Up) in m/s (NOTE: Positive is DOWN in NED frame)
    """
    
    # The message requires 11 mandatory parameters for position/velocity/accel/yaw
    master.mav.set_position_target_local_ned_send(
        0,                                   # time_boot_ms (not used)
        master.target_system, master.target_component, 
        MAV_FRAME_BODY_NED_NUM,              # Frame is relative to the drone's body
        MASK_SET_VELOCITY_ONLY,              # Only use velocity fields
        0, 0, 0,                             # 1-3. X, Y, Z Position (ignored by mask)
        vx, vy, vz,                          # 4-6. VX, VY, VZ Velocity (used)
        0, 0, 0,                             # 7-9. AX, AY, AZ Acceleration (ignored)
        0, 0                                 # 10-11. Yaw, Yaw Rate (ignored)
    )

def land_drone(master):
    """Sends a MAV_CMD_NAV_LAND command."""
    print("\nInitiating landing sequence...")
    # Set all velocities to zero before landing to ensure stability
    with state_lock:
        global desired_vx, desired_vy, desired_vz
        desired_vx, desired_vy, desired_vz = 0.0, 0.0, 0.0

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Land command sent. Drone is descending.")
    
    # Wait for disarm or change to land mode before continuing
    while master.motors_armed():
        time.sleep(0.5)
    print("Drone landed and disarmed.")

def disarm_drone(master):
    """Disarms the motors if the drone is on the ground."""
    print("Disarming motors...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Motors disarmed. Safe to exit.")

# --- CONTINUOUS SENDER THREAD ---

def velocity_sender_loop(master):
    """Continuously sends the current desired velocity to prevent timeout."""
    global desired_vx, desired_vy, desired_vz
    
    # Loop continuously while the script is running
    while master.running:
        try:
            with state_lock:
                vx = desired_vx
                vy = desired_vy
                vz = desired_vz
            
            # Only send if the drone is armed and in GUIDED mode
            if master.motors_armed() and master.flightmode == 'GUIDED':
                send_velocity_command(master, vx, vy, vz)
            
            # Wait for the next send interval
            time.sleep(1.0 / SEND_RATE_HZ)

        except Exception:
            # Silence internal pymavlink errors during the loop
            pass
            
# --- MAIN APPLICATION LOGIC ---

def main_loop(master):
    global desired_vx, desired_vy, desired_vz

    # Start the continuous velocity sender in a separate thread
    master.running = True
    sender_thread = threading.Thread(target=velocity_sender_loop, args=(master,), daemon=True)
    sender_thread.start()
    
    # Key mapping introduction
    print("\n--- Controls ---")
    print(" T: Arm and Takeoff (Now using Velocity Climb)")
    print(" W/S: Forward/Backward (Body Frame)")
    print(" A/D: Left/Right (Body Frame)")
    print(" R/F: Altitude Up/Down")
    print(" H: Hover (Stop all movement)")
    print(" Q: Land")
    print(" X: Exit and Disarm")
    print("----------------\n")

    while True:
        try:
            # Print status before taking input
            alt_status = get_current_altitude(master)
            armed_status = "ARMED" if master.motors_armed() else "DISARMED"
            
            user_input = input(f"[{armed_status} | Alt: {alt_status} | Mode: {master.flightmode}] > ").lower()
            
            # Initialize velocities to zero only if the user is inputting a movement command
            vx, vy, vz = 0.0, 0.0, 0.0
            
            # Check if this is a movement command that requires updating desired velocities
            if user_input in ['w', 's', 'a', 'd', 'r', 'f', 'h']:
                # The continuous sender needs the desired state, so we update the globals
                if user_input == 'w':
                    vx = CRUISE_SPEED_MPS
                    print("-> Moving Forward...")
                elif user_input == 's':
                    vx = -CRUISE_SPEED_MPS
                    print("-> Moving Backward...")
                elif user_input == 'd':
                    vy = CRUISE_SPEED_MPS
                    print("-> Moving Right...")
                elif user_input == 'a':
                    vy = -CRUISE_SPEED_MPS
                    print("-> Moving Left...")
                elif user_input == 'r':
                    # NED frame: Negative VZ is UP
                    vz = -VERTICAL_SPEED_MPS
                    print("-> Gaining Altitude (Up)...")
                elif user_input == 'f':
                    # NED frame: Positive VZ is DOWN
                    vz = VERTICAL_SPEED_MPS
                    print("-> Reducing Altitude (Down)...")
                elif user_input == 'h':
                    # All velocities are zero
                    print("-> Hover (Stopping movement)...")
                
                # Apply new velocity state
                with state_lock:
                    desired_vx = vx
                    desired_vy = vy
                    desired_vz = vz

            # T: Arm and Takeoff
            elif user_input == 't':
                if not master.motors_armed():
                    # CALL THE NEW VELOCITY-BASED TAKEOFF
                    guided_velocity_takeoff(master, TARGET_ALTITUDE_METERS)
                    # After successful takeoff, ensure the background loop is sending 0 velocity (hover)
                    with state_lock:
                        desired_vx, desired_vy, desired_vz = 0.0, 0.0, 0.0
                else:
                    print(f"-> Motors are already armed/airborne.")

            # Q: Land
            elif user_input == 'q':
                land_drone(master)
                
            # X: Exit
            elif user_input == 'x':
                master.running = False
                if master.motors_armed():
                    disarm_drone(master)
                print("-> Exiting application...")
                break
                
            else:
                print("-> Invalid command. Please try again.")

        except KeyboardInterrupt:
            master.running = False
            break
        except Exception as e:
            print(f"\n[ERROR] An unexpected error occurred: {e}")
            break

def main():
    """Initializes connection and runs the main loop."""
    master = None
    try:
        # Create a MAVLink connection
        master = mavutil.mavlink_connection(CONNECTION_STRING)
        
        # Add 'running' flag to the master object for thread control
        master.running = False

        wait_for_heartbeat(master)
        main_loop(master)

    except Exception as e:
        print(f"\n--- FATAL ERROR ---")
        print(f"Could not connect to the drone or an error occurred: {e}")
        print(f"Ensure SITL/MAVProxy is running and connected to {CONNECTION_STRING}")
        sys.exit(1)
    finally:
        # Ensure the sender thread is signaled to stop
        if master and master.running:
            master.running = False
        # Give the sender thread a moment to shut down gracefully
        time.sleep(1)


if __name__ == "__main__":
    main()
