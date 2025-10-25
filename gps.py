#!/usr/bin/env python3
# Fly without GPS using Raspberry Pi + Pixhawk + PyMavlink
# Mode: AltHold (uses barometer only)

from pymavlink import mavutil
import time

# ------------------------------------------------------------
# 1️⃣ Connect to Pixhawk
# ------------------------------------------------------------
# Replace '/dev/ttyACM0' with your port and 115200 with your baudrate
#master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master = mavutil.mavlink_connection('udpin:localhost:14551')
# Wait for heartbeat (system is ready)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# ------------------------------------------------------------
# 2️⃣ Set flight mode to AltHold (non-GPS)
# ------------------------------------------------------------
# Mode numbers depend on your firmware. For ArduCopter:
#   0 = Stabilize, 2 = AltHold, 3 = Auto, 5 = Loiter, etc.
ALT_HOLD_MODE = 2

master.set_mode(ALT_HOLD_MODE)
print("Mode set to AltHold")

# ------------------------------------------------------------
# 3️⃣ Arm the drone
# ------------------------------------------------------------
# Send arm command (MAV_CMD_COMPONENT_ARM_DISARM = 400)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Arming motors...")
time.sleep(3)

# ------------------------------------------------------------
# 4️⃣ Take off manually (increase throttle slowly)
# ------------------------------------------------------------
# In AltHold mode, you control climb using throttle.
# If you want autonomous lift, send a small vertical velocity.
print("Taking off...")

# Send small upward velocity (0, 0, -1 m/s in NED)
for _ in range(30):  # ~3 seconds
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # ignore pos, use velocity
        0, 0, 0,              # position (ignored)
        0, 0, -1,             # velocity: up
        0, 0, 0,              # accel (ignored)
        0, 0                  # yaw, yaw_rate (ignored)
    )
    time.sleep(0.1)

# ------------------------------------------------------------
# 5️⃣ Move forward for a few seconds
# ------------------------------------------------------------
print("Moving forward...")
for _ in range(50):  # ~5 seconds
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        1, 0, 0,             # move forward 1 m/s
        0, 0, 0,
        0, 0
    )
    time.sleep(0.1)

# ------------------------------------------------------------
# 6️⃣ Land (reduce thrust)
# ------------------------------------------------------------
print("Landing...")
for _ in range(40):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0.5,           # small downward velocity
        0, 0, 0,
        0, 0
    )
    time.sleep(0.1)

# ------------------------------------------------------------
# 7️⃣ Disarm
# ------------------------------------------------------------
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Disarmed. Mission complete.")
