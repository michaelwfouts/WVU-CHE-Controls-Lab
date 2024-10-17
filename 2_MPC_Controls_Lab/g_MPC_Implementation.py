import serial # pyserial
import time
import helper_functions as hf
import pandas as pd
from datetime import datetime
import numpy as np
import joblib

# Load in models
gp_model_dh1 = joblib.load('6 Online Update Controls/Arduino/4_MPC_Controls_Lab/GP_Models/gp_model_dh1.pkl')
gp_model_dh2 = joblib.load('6 Online Update Controls/Arduino/4_MPC_Controls_Lab/GP_Models/gp_model_dh2.pkl')

# Configuration
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
TIME_STEP = 3  # second
INITIAL_TEST_COUNT = 3
PWM_STEP = 5
CSV_PATH = '6 Online Update Controls/Arduino/4_MPC_Controls_Lab/data.csv'

# Set up the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)  # Open serial port on appropriate COM port at 9600 baud

# Initialize an empty DataFrame to store collected data
data = pd.DataFrame(columns=['Timestamp', 'PWM Value', 'Height Tank 1', 'Height Tank 2'])

# Initial test: Send maximum PWM value to ensure system is working
print("Initializing system...")
for i in range(INITIAL_TEST_COUNT):
    hf.send_pwm_value(ser, 255)  # Send maximum PWM value (255) for full open valve
    print(hf.read_from_serial(ser))  # Read and print the response
    time.sleep(5)  

height_tank_2, height_tank_1 = hf.read_from_serial(ser)
x = np.array([height_tank_1, height_tank_2])

# System parameters
Q2 = np.array([[1]])  # Penalize only Tank 2's height
R = np.array([[0.1]])  # Penalize input effort
Np = 10  # Prediction horizon
Nc = 3   # Control horizon
x0 = x  # Initial water heights in both tanks
x_ref = np.array([None, 350])  # Desired water height for Tank 2

# Initialize the MPC controller with trained models
mpc = hf.MPC(gp_model_dh1, gp_model_dh2, Q2, R, Np, Nc, x0, TIME_STEP)

print('Started MPC Control')
start_time = datetime.now()

try:
    # Continuous monitoring and data collection loop
    while True:
        # Read start loop time
        start_time_loop = datetime.now()
        # Read tank heights (A0 is height of tank 2, A1 is height of tank 1 on the Arduino)
        height_tank_2, height_tank_1 = hf.read_from_serial(ser)
        x = np.array([height_tank_1, height_tank_2])

        # Determine what the height of the tank should be
        u = mpc.apply_control(x, x_ref)

        if height_tank_1 is not None and height_tank_2 is not None:
            # Send updated PWM value to Arduino
            hf.send_pwm_value(ser, u, sleep_time=0.1)

            # Record data
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            new_data = pd.DataFrame([{
                'Timestamp': timestamp,
                'PWM Value': u,
                'Height Tank 1': height_tank_1,
                'Height Tank 2': height_tank_2
            }])
            data = pd.concat([data, new_data], ignore_index=True)

            print(f"Recorded: {timestamp} - PWM: {u}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2}")
        
        # print(hf.read_from_serial(ser))  # Print current readings for monitoring
        wait = True
        end_time_loop = datetime.now()
        while wait == True:
            if (end_time_loop - start_time_loop).total_seconds() < TIME_STEP:
                time.sleep(0.1) # Wait for specified time step between readings
                end_time_loop = datetime.now()
            else:
                wait = False

        current_time = datetime.now()

        if (current_time - start_time).total_seconds() > 100:
            # Generate filename with current date and timestep
            timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            filename = f"dataframe_{timestamp}.csv"
            
            # Save the dataframe to CSV
            data.to_csv(filename, index=False)
            start_time = current_time

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