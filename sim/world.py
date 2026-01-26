import serial
import pygame
import math

# --- CONFIG ---
SERIAL_PORT = 'COM4'
ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.01)

# Robot Physicals (Matching your LemLib config)
TRACK_WIDTH = 10.8
WHEEL_DIAMETER = 3.25
MAX_SPEED_IPS = (450 / 60.0) * (math.pi * 3.25)
SCALE = 4.0 # pixels per inch

x, y, theta = 400.0, 400.0, 0.0 # Starting at center (100, 100 in inches)
v_l, v_r = 0, 0

running = True
while running:
    # 1. READ VOLTAGES FROM BRAIN (if available)
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        try:
            v_l, v_r = map(float, line.split(','))
            # Basic Kinematics update
            vel_l = (v_l / 12000.0) * MAX_SPEED_IPS
            vel_r = (v_r / 12000.0) * MAX_SPEED_IPS
            linear = (vel_l + vel_r) / 2.0
            angular = (vel_r - vel_l) / TRACK_WIDTH
            theta += angular * 0.02
            x += (linear * math.cos(theta) * 0.02) * SCALE
            y -= (linear * math.sin(theta) * 0.02) * SCALE
        except: pass

    # 2. ALWAYS SEND POSE TO BRAIN
    # Format: "x,y,theta\n" (Inches and Radians)
    pose_msg = f"{(x/SCALE):.2f},{(y/SCALE):.2f},{theta:.3f}\n"
    ser.write(pose_msg.encode())
    print(f"Feeding Brain: {pose_msg.strip()}") # Debug view

    # 3. VISUALIZE
    # [Pygame drawing code from previous step goes here]
    # ...
