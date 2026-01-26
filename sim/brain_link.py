import serial
import time
import random

USER_PORT = 'COM5' 
BAUD_RATE = 115200

try:
    # Open the port ONCE outside the loop
    ser = serial.Serial(USER_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 

    while True:
        num = round(random.random(), 3)
        # 1. Use str() for conversion
        # 2. Add \n so std::cin knows the message is finished
        message = str(num) + "\n"
        
        ser.write(message.encode())
        print(f"Sent: {message.strip()}")

        # Read the confirmation from the Brain
        response = ser.readline().decode().strip()
        if response:
            print(f"Response: {response}")

        time.sleep(0.05) # Don't flood the Brain too fast

except Exception as e:
    print(f"Error: {e}")
finally:
    # Only close when you exit the loop (Ctrl+C)
    if 'ser' in locals() and ser.is_open:
        ser.close()
