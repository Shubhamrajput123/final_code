# This script controls a real drone using pymavlink via a serial connection
# for incremental takeoff, landing, and directional movement.
# --- This version uses numerical MAVLink constants for robust error prevention. ---

import time
import sys
from pymavlink import mavutil
import termios, tty, select, threading, math, os


# --- MAVLink Numerical Constants for Robustness ---
# MAV_CMD_COMPONENT_ARM_DISARM (400)
MAV_CMD_ARM_DISARM = 400
# MAV_CMD_NAV_TAKEOFF (22)
MAV_CMD_TAKEOFF = 22
# MAV_FRAME_GLOBAL_RELATIVE_ALT (3) - For Altitude control
MAV_FRAME_GLOBAL_RELATIVE_ALT_NUM = 3 
# MAV_FRAME_LOCAL_OFFSET_NED (7) - For directional movement
MAV_FRAME_BODY_OFFSET_NED_NUM = 8
MASK_SET_VELOCITY_ONLY = 1479
MASK_SET_VELOCITY_AND_YAW = 3015
# MASK for Setting Altitude only (Z Position)
# Ignores X, Y, all velocities, all accelerations, and yaw/yaw rate.
MASK_SET_ALTITUDE_ONLY = 3579
# MASK for Setting Horizontal Position only (X and Y Position)
# Ignores Z, all velocities, all accelerations, and yaw/yaw rate.
MASK_SET_HORIZONTAL_ONLY = 4088
VELOCITY_M_S=3.0
# --- Configuration for Real Drone (Serial Connection) ---
SERIAL_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 57600            
TARGET_STEP_METERS = 2.0      # Altitude increment per 'T' press
MOVEMENT_STEP =2.0           # Horizontal movement per N/S/E/W press
current_target_altitude = 0.0 # Tracks the drone's intended altitude

# --- Utility Functions (Numerical Command Updates) ---

def wait_for_heartbeat(master):
    """Waits for a MAVLink heartbeat to confirm connection."""
    print(f"Connecting to MAVLink endpoint: {SERIAL_PORT} @ {BAUD_RATE}...")
    master.wait_heartbeat()
    print(f"Heartbeat received! System: {master.target_system}, Component: {master.target_component}")

def set_guided_mode(master):
    """Sets the flight mode to GUIDED (required for sending direct commands)."""
    mode = 'GUIDED'
    try:
        mode_id = master.mode_mapping()[mode]
    except KeyError:
        print(f"Error: Mode '{mode}' not found in vehicle's mode map.")
        return

    master.set_mode(mode_id)

    print(f"Attempting to set mode to {mode}...")
    timeout = time.time() + 5 
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
        if msg and mavutil.mode_string_v10(msg) == mode:
            print(f"Mode successfully set to {mode}.")
            break
        if time.time() > timeout:
            print(f"Warning: Mode change to {mode} timed out. Proceeding anyway...")
            break
        time.sleep(0.1)

def arm_vehicle(master):
    """Arms the drone if it is not already armed. Uses numerical MAV_CMD_ARM_DISARM (400)."""
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAV_CMD_ARM_DISARM, # Use numerical command ID 400
        0, 1, 0, 0, 0, 0, 0, 0 
    )
    print("Waiting for arming...")
    master.motors_armed_wait() 
    print("Motors armed successfully!")

def land_vehicle(master):                                        
    """Sets the mode to AUTO.LAND."""
    global current_target_altitude
    
    print("Setting mode to LAND...")
    master.set_mode('LAND')
    
    current_target_altitude = 0.0
    print("Land command sent. Waiting for vehicle to disarm...")
    
    master.motors_disarmed_wait()
    print("Vehicle landed and disarmed. Control complete.")
    master.close()
# --- Altitude Command (Numerical Mask Applied) ---

def send_takeoff_command(master, altitude):
    """Sends the takeoff command or altitude adjustment."""
    global current_target_altitude

    if altitude <= 0:
        print("Error: Target altitude must be greater than 0.")
        return

    # 1. Arm if first takeoff
    if current_target_altitude == 0.0:
         arm_vehicle(master)
    
    current_target_altitude = altitude
    
    print(f"Setting target altitude to {current_target_altitude} meters.")
    
    # 2. Send Command
    if altitude == TARGET_STEP_METERS:
        # Use MAV_CMD_NAV_TAKEOFF (22) for the initial lift-off
        print("Sending initial MAV_CMD_NAV_TAKEOFF (22)...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            MAV_CMD_TAKEOFF, # Use numerical command ID 22
            0, 0, 0, 0, 0, 0, 0, 
            altitude # param7: Target altitude
        )
    else:
        # Use SET_POSITION_TARGET_GLOBAL_INT for subsequent altitude changes
        print("Sending new altitude target via SET_POSITION_TARGET_GLOBAL_INT...")
        master.mav.set_position_target_global_int_send(
            0, # time_boot_ms
            master.target_system, 
            master.target_component, 
            MAV_FRAME_BODY_OFFSET_NED_NUM, # Use numerical frame 3
            MASK_SET_VELOCITY_ONLY, # Use numerical mask 3579
            0, # lat_int (ignored)
            0, # lon_int (ignored)
            0, # alt_int (in mm)
            0, 0, -VELOCITY_M_S, # vx, vy, vz (Velocities - ignored by mask)
            0, 0, 0, # afx, afy, afz (Accelerations - required, ignored by mask)
            0, # yaw (ignored by mask)
            0  # yaw_rate (ignored by mask)
        )
        time.sleep(altitude/VELOCITY_M_S)
        master.mav.set_position_target_local_ned_send(0, master.target_system, master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,
        MASK_SET_VELOCITY_ONLY,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0
        )
    print(f"Target set to {current_target_altitude}m. Monitoring telemetry...")
    time.sleep(2) # Give a moment for the command to register

# --- Movement Command (Numerical Mask Applied) ---

def send_movement_command(master, direction):
    """Sends a local velocity command (NED) to move at a fixed speed, then stop."""
    global MOVEMENT_STEP # NOTE: MOVEMENT_STEP is no longer needed here

    if current_target_altitude <= 0:
        print("Movement ignored. Please press 'T' first to take off.")
        return

    # North, East, Down (NED) frame: North is +X, East is +Y
    north_vel = 0.0
    east_vel = 0.0
    yaw_degree = 0.0
    
    if direction == 'W':
        north_vel = VELOCITY_M_S
        yaw_degree = 0.0
    elif direction == 'S':
        north_vel = -VELOCITY_M_S
        yaw_degree = 0.0
    elif direction == 'D':
        east_vel = VELOCITY_M_S
        yaw_degree = 90.0
    elif direction == 'A':
        east_vel = -VELOCITY_M_S
        yaw_degree = 90.0
    else:
        print(f"Error: Unknown direction '{direction}'.")
        return

    print(f"Command sent: Move at {VELOCITY_M_S} m/s to the {direction} (N:{north_vel}m/s, E:{east_vel}m/s).")
    
    # 1. Send the movement command (velocity setpoint)
    master.mav.set_position_target_local_ned_send(
        0, # time_boot_ms
        master.target_system, master.target_component, 
        MAV_FRAME_BODY_OFFSET_NED_NUM, # Use numerical frame 7
        MASK_SET_VELOCITY_AND_YAW, # Use numerical mask 1479
        0, # X: Position (ignored by mask)
        0, # Y: Position (ignored by mask)
        0, # Z: Position (ignored by mask)
        north_vel, # VX: North Velocity (m/s)
        east_vel,  # VY: East Velocity (m/s)
        0,         # VZ: Down Velocity (m/s)
        0, 0, 0, # Accelerations (ignored)
        yaw_degree, 0     # Yaw/Yaw Rate (ignored)
    )

    # 2. Wait for the desired movement time (e.g., 5 seconds)
    time.sleep(5) 
    
    # 3. Send a STOP command (zero velocity) to prevent continuous movement
    print("Sending STOP command (zero velocity).")
    master.mav.set_position_target_local_ned_send(
        0, # time_boot_ms
        master.target_system, master.target_component, 
        MAV_FRAME_BODY_OFFSET_NED_NUM, # Use numerical frame 7
        MASK_SET_VELOCITY_ONLY, # Use numerical mask 1479
        0, 0, 0, 
        0, 0, 0, # VX, VY, VZ = 0
        0, 0, 0, 
        0, 0
    )
    # Give the drone a moment to register the stop command
    time.sleep(1)
def send_yaw_command(master, direction):
    """
    Rotates the drone left ('A') or right ('D') in place (no translation).
    Positive yaw_rate = turn right, Negative = turn left.
    """
    yaw_rate = 0.0

    if direction == 'A':     # turn left
        yaw_rate = -0.5      # radians per second (about 30°/s)
    elif direction == 'D':   # turn right
        yaw_rate = 0.5
    else:
        print(f"Unknown yaw direction: {direction}")
        return

    print(f"Turning {'left' if yaw_rate < 0 else 'right'} at {yaw_rate} rad/s")

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,   # 8: relative to current heading
        MASK_SET_VELOCITY_ONLY,          # velocity-only control
        0, 0, 0,                         # position ignored
        0, 0, 0,                         # no linear motion
        0, 0, 0,
        0, yaw_rate                      # yaw_rate (rad/s)
    )

    time.sleep(2.5)  # rotate for 2 seconds

    # Stop rotation
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,
        MASK_SET_VELOCITY_ONLY,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    print("Yaw stop.")
def send_descend_command(master, step=4.0):
    """Decreases the current altitude by 'step' meters safely."""
    global current_target_altitude

    if current_target_altitude <= step:
        print("Too low to descend further safely.")
        return

    new_altitude = current_target_altitude - step
    print(f"Descending from {current_target_altitude}m to {new_altitude}m...")

    # Use SET_POSITION_TARGET_GLOBAL_INT with downward velocity
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,
        MASK_SET_VELOCITY_ONLY,
        0, 0, 0,         # lat/lon ignored
        0, 0, 1,  # vz positive = descend (NED frame: +down)
        0, 0, 0,
        0, 0
    )

    # Wait for descent duration
    time.sleep(step / 1)

    # Stop descending
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,
        MASK_SET_VELOCITY_ONLY,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    current_target_altitude = new_altitude
    print(f"New target altitude: {current_target_altitude}m")
def get_key():
    """Non-blocking single key capture using termios."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

rotation_active = False
rotation_direction = None

def send_yaw_rate(master, yaw_rate):
    """Send yaw rotation command."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM,
        0b0000011111111000,  # ignore everything except yaw_rate
        0, 0, 0,             # position ignored
        0, 0, 0,             # velocity ignored
        0, 0, 0,             # accel ignored
        0, yaw_rate          # yaw, yaw_rate
    )

def stop_yaw(master):
    """Stops all yaw motion."""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        MAV_FRAME_BODY_OFFSET_NED_NUM, MASK_SET_VELOCITY_ONLY,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )

def is_key_pressed(char):
    """Check if a specific key is still pressed (for R/F rotation)."""
    import termios, sys, select, tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            c = sys.stdin.read(1)
            if c.lower() == char.lower():
                return True
        return False
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    global current_target_altitude

    try:
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, autoreconnect=True)
        wait_for_heartbeat(master)
        set_guided_mode(master)

        print("\n--- Drone Control Ready ---")
        print("T: Ascend | G: Descend | Q: LAND")
        print("W/S: Forward/Backward | A/D: Left/Right movement")
        print("R: Rotate Right (hold) | F: Rotate Left (hold)")
        print("X: Exit\n")

        rotating = False
        last_key = ''

        while True:
            key = get_key().upper()

            if key:
                last_key = key

                # --- Ascend ---
                if key == 'T':
                    new_target = current_target_altitude + TARGET_STEP_METERS
                    send_takeoff_command(master, new_target)

                # --- Descend ---
                elif key == 'G':
                    send_descend_command(master, step=TARGET_STEP_METERS)

                # --- Movement ---
                elif key in ('W', 'S'):
                    send_movement_command(master, key)
                elif key in('A','D'):
                    send_yaw_command(master,key)
                # --- Land ---
                elif key == 'Q':
                    land_vehicle(master)
                    break

                # --- Continuous Yaw Right ---
                elif key == 'R':
                    if not rotating:
                        rotating = True
                        print("Holding R → continuous right rotation")
                        while True:
                            yaw_rate = 0.17  # ≈ 1°/s
                            master.mav.set_position_target_local_ned_send(
                                0, master.target_system, master.target_component,
                                MAV_FRAME_BODY_OFFSET_NED_NUM, MASK_SET_VELOCITY_ONLY,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate
                            )
                            time.sleep(1)
                            # Check if still pressing R
                            if not is_key_pressed('r'):
                                print("Stop rotating right.")
                                stop_yaw(master)
                                rotating = False
                                break

                # --- Continuous Yaw Left ---
                elif key == 'F':
                    if not rotating:
                        rotating = True
                        print("Holding F → continuous left rotation")
                        while True:
                            yaw_rate = -0.17  # ≈ 1°/s
                            master.mav.set_position_target_local_ned_send(
                                0, master.target_system, master.target_component,
                                MAV_FRAME_BODY_OFFSET_NED_NUM, MASK_SET_VELOCITY_ONLY,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate
                            )
                            time.sleep(1)
                            # Check if still pressing F
                            if not is_key_pressed('f'):
                                print("Stop rotating left.")
                                stop_yaw(master)
                                rotating = False
                                break

                elif key == 'X':
                    print("Exiting script.")
                    break

                else:
                    print("Invalid key.")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting safely...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'master' in locals():
            master.close()
            print("MAVLink connection closed.")

if __name__ == '__main__':
    main()
