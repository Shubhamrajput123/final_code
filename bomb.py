from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('dev/ttyAMA0', baud = 57600)
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")
SERVO_1_ID = 5
SERVO_2_ID = 6
PWM_MAX = 2000

def set_servo(servo_id, pwm_value):
    master.mav.commmand_long_send()(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_id,
        pwm_value,
        0,0,0,0,0
    )
    print(f"sent command to servo {servo_id} with pwm {pwm_value}us")

try:
    set_servo(SERVO_1_ID, PWM_MAX)
    time.sleep(0.5)
    set_servo(SERVO_2_ID,PWM_MAX)
except Exception as e:
    print(f"Error: {e}")
    master.close()