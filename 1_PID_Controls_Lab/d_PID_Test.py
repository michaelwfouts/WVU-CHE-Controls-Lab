import serial # pyserial
import pandas as pd
import time
from datetime import datetime
import helper_functions as hf

# Set up the serial connection
ser = serial.Serial('COM3', 9600)  # Adjust the port as needed

# Initialize an empty DataFrame to store collected data
data = pd.DataFrame(columns=['Timestamp', 'PWM Value', 'Height Tank 1', 'Height Tank 2'])

# Set initial values
pwm_value = 255 # Initial Charging Signal to Arduino
time_step = 3 # second
height_tank_2 = None # Value set here to ensure value is properly read from Arduino

# Values from Initial Ziegler Nichols Tuning, but can be changed to fit needs
Controller = hf.PIDController(Kp = 8.22, 
                              Ki = 0.288, 
                              Kd = 58.57, 
                              setpoint = 350)


# Send charging signal to arduino to ensure it is working
for i in range(5):
    hf.send_pwm_value(ser, pwm_value)
    print(hf.read_from_serial(ser))
    time.sleep(time_step)

# Control Loop
try:
    while True:
        height_tank_2, height_tank_1 = hf.read_from_serial(ser)
        if height_tank_2 is not None:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # Create a new DataFrame row for the current record
            new_data = pd.DataFrame([{'Timestamp': timestamp, 'PWM Value': pwm_value, 'Height Tank 1': height_tank_1, 'Height Tank 2': height_tank_2}])
            # Concatenate the new data with the existing DataFrame
            data = pd.concat([data, new_data], ignore_index=True)
            print(f"Recorded: {timestamp} - PWM: {pwm_value}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2}")
            # Update Control Signal
            pwm_value = Controller.update(height_tank_2, time_step)
            pwm_value = max(1, min(pwm_value, 255))
            hf.send_pwm_value(ser, pwm_value)
            # Wait for next evaluation and reset height_tank_2
            time.sleep(time_step)
            height_tank_2 = None

except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    ser.close()  # Close the serial port
    data.to_csv('pid_data.csv', index=False)  # Save the DataFrame to a CSV file
    print("Data saved to 'data.csv'.")