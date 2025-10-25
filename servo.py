from pymavlink import mavutil
import pygame
import time

# --- Connect to Pixhawk ---
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("✅ Connected to Pixhawk")

# --- Initialize Pygame for keyboard ---
pygame.init()
screen = pygame.display.set_mode((200, 100))
pygame.display.set_caption("Servo Control")

# --- Servo Parameters ---
SERVO_CHANNEL = 9      # AUX1 = channel 9
SERVO_MIN = 1000       # microseconds
SERVO_MAX = 2000       # microseconds
angle_180 = SERVO_MAX  # position for 180°
angle_0 = SERVO_MIN    # position for 0°

def set_servo(channel, pwm_value):
    """Send command to set servo PWM"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )

current_angle = 0

print("Press G to move servo to 180°, press H to move back to 0°")

# --- Main loop ---
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_g:
                print("➡ Rotating to 180°")
                set_servo(SERVO_CHANNEL, angle_180)
                current_angle = 180
                time.sleep(1)
            
            elif event.key == pygame.K_h:
                print("⬅ Rotating to 0°")
                set_servo(SERVO_CHANNEL, angle_0)
                current_angle = 0
                time.sleep(1)
            
            elif event.key == pygame.K_q:
                running = False

pygame.quit()
print("Program exited.")
