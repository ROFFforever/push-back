import serial
import pygame
import math
import re
import time

# --- CONFIG ---
USER_PORT = 'COM5'
BAUD_RATE = 115200
hertz = 50.0 #50hz

# Robot Physicals (Matching your LemLib config)
TRACK_WIDTH = 10.8
MAX_VOLTS = 12000
DRIVETRAIN_RPM = 450
WHEEL_DIAMETER = 3.25
MAX_SPEED_IPS = (DRIVETRAIN_RPM / 60.0) * (math.pi * WHEEL_DIAMETER)
SCALE = 4.0 # pixels per inch
DT = 1.0 / hertz

x, y, theta = 0, 0, 0 #theta in radians
v_l, v_r = 0, 0

running = True
def update_physics(v_left, v_right, x, y, theta):
    # 1. Convert volts to Inches Per Second
    # (Voltage / 12000) * (RPM / 60) * Circumference
    ips_l = (v_left / MAX_VOLTS) * MAX_SPEED_IPS
    ips_r = (v_right / MAX_VOLTS) * MAX_SPEED_IPS

    # 2. Calculate Forward Velocity and Angular Velocity (Rotation)
    # Track width from your code: 10.8 inches
    v_linear = (ips_l + ips_r) / 2.0
    v_angular = (ips_r - ips_l) / TRACK_WIDTH # Radians per second

    # 3. Update Position (Odometry)
    # Move in the direction of current theta
    new_x = x + (v_linear * math.cos(theta) * DT)
    new_y = y + (v_linear * math.sin(theta) * DT)
    new_theta = theta + (v_angular * DT)

    return new_x, new_y, new_theta
try:
    # Open the port ONCE outside the loop
    ser = serial.Serial(USER_PORT, BAUD_RATE, timeout=1)
    time.sleep(1) 
    while running:
        #send robot position
        message = str(x) + "," + str(y) + "," + str(theta) + "\n" #format to be x,y,theta
        ser.write(message.encode()) #send coords to vex brain
        print(f"Sent: {message}") #message sent, should be like 10.4,13.2,73.4
        response = ser.readline().decode().strip()
        if response:
    # 1. CLEANING: Remove EVERYTHING except numbers, commas, dots, and minus signs
            response = re.sub(r'[^0-9,.\-]', '', response)

        try:
            # 2. SPLIT & CAST
            parts = response.split(",")
            if len(parts) >= 2:
                lV = float(parts[0])
                rV = float(parts[1])

                # Physics update
                newx, newy, newtheta = update_physics(lV, rV, x, y, theta)
                x, y, theta = newx, newy, newtheta
                x = round(x,3)
                y = round(y,3)
                theta = round(theta,3)

                # 3. PRINT FIX: Use f-strings for numbers (prevents "can only concatenate str" errors)
                print(f"Pos: {x:.2f} {y:.2f} {theta:.2f}")
        except ValueError as e:
            print(f"Skipping malformed data: {response}")

        time.sleep(1 / hertz) # Don't flood the Brain too fast 25hz shoulod be 
   
        
except Exception as e:
    print(f"Error: {e}")
finally:
    # Only close when you exit the loop (Ctrl+C)
    if 'ser' in locals() and ser.is_open:
        ser.close()
