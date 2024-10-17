"""
Data Collection for Machine Learning Model Training in Model Predictive Control (MPC)
This methodology collects data by slowly opening and closing the control valve.

This script collects data from a two-tank system controlled by an Arduino.
The data will be used to train a machine learning model for MPC implementation.

The script does the following:
1. Establishes a serial connection with an Arduino.
2. Initializes the system by sending maximum PWM values.
3. Continuously collects data while oscillating the PWM value.
4. Saves the collected data to a CSV file.
"""

import serial # pyserial
import time
import helper_functions as hf
import pandas as pd
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
TIME_STEP = 1  # second
INITIAL_TEST_COUNT = 5
PWM_STEP = 5
CSV_PATH = '6 Online Update Controls/Arduino/4_MPC_Controls_Lab/data.csv'
MIN_PWM = 90 # PWM values below 0-90 are all effectively closed for this specific application
INITIAL_PWM = 240 # Starting value from MIN_PWM to 255 to get variety of data

# Set up the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)  # Open serial port on appropriate COM port at 9600 baud

# Initialize an empty DataFrame to store collected data
data = pd.DataFrame(columns=['Timestamp', 'PWM Value', 'Height Tank 1', 'Height Tank 2'])

# Initial test: Send maximum PWM value to ensure system is working
print("Initializing system...")
for i in range(INITIAL_TEST_COUNT):
    hf.send_pwm_value(ser, 255)  # Send maximum PWM value (255) for full open valve
    print(hf.read_from_serial(ser))  # Read and print the response
    time.sleep(TIME_STEP)  

print('Started Data Collection')

try:
    # Initialize variables for PWM oscillation
    pwm_value = INITIAL_PWM
    direction = 1  # 1 for increasing, -1 for decreasing

    # Continuous monitoring and data collection loop
    while True:
        # Read tank heights (A0 is height of tank 2, A1 is height of tank 1 on the Arduino)
        height_tank_2, height_tank_1 = hf.read_from_serial(ser)

        # Update and oscillate PWM value
        pwm_value += PWM_STEP * direction
        pwm_value = max(MIN_PWM, min(255, pwm_value))  # Ensure PWM value stays within 0-255
        
        # Change direction if limits are reached
        if pwm_value >= 255 or pwm_value <= MIN_PWM:
            direction *= -1

        if height_tank_1 is not None and height_tank_2 is not None:
            # Send updated PWM value to Arduino
            hf.send_pwm_value(ser, pwm_value, sleep_time=0.1)

            # Record data
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            new_data = pd.DataFrame([{
                'Timestamp': timestamp,
                'PWM Value': pwm_value,
                'Height Tank 1': height_tank_1,
                'Height Tank 2': height_tank_2
            }])
            data = pd.concat([data, new_data], ignore_index=True)

            print(f"Recorded: {timestamp} - PWM: {pwm_value}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2}")
        
        print(hf.read_from_serial(ser))  # Print current readings for monitoring
        time.sleep(TIME_STEP)  # Wait for specified time step between readings

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Clean up and save data
    ser.close()  # Close the serial port
    if not data.empty:
        data.to_csv(CSV_PATH, index=False)  # Save the DataFrame to a CSV file
        print(f"Data saved to '{CSV_PATH}'.")
    else:
        print("No data was collected.")