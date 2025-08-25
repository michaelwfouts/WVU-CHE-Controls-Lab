import serial # pyserial
import pandas as pd
import time
from datetime import datetime
import helper_functions as hf

# --- Configuration Parameters ---
SERIAL_PORT = 'COM3'  # Adjust the port as needed
BAUD_RATE = 9600
INITIAL_PWM = 255     # Initial Charging Signal to Arduino
TIME_STEP = 3         # Seconds between readings/updates
DATA_COLLECTION_DURATION = 600 # seconds (e.g., 600 seconds = 10 minutes)

# Values from Initial Ziegler Nichols Tuning, but can be changed to fit needs
# The setpoint is now explicitly included in the DataFrame
CONTROLLER_KP = 10
CONTROLLER_KI = 0
CONTROLLER_KD = 0
SETPOINT_VALUE = 350

# --- PID Controller Setup ---
Controller = hf.PIDController(Kp=CONTROLLER_KP,
                              Ki=CONTROLLER_KI,
                              Kd=CONTROLLER_KD,
                              setpoint=SETPOINT_VALUE)

# --- Serial Connection Setup ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    print("Please check if the Arduino is connected and the port is correct.")
    exit()

# Initialize an empty DataFrame to store collected data
# Added 'Setpoint' column to the DataFrame
data = pd.DataFrame(columns=['Timestamp', 'Setpoint', 'PWM Value', 'Height Tank 1', 'Height Tank 2'])

# Initialize variables
height_tank_2 = None # Value set here to ensure value is properly read from Arduino
start_time = time.time() # Record the start time for the time limit

print("Starting PID control loop...")

# --- Control Loop ---
try:
    # Loop continues as long as the elapsed time is less than the specified duration
    while (time.time() - start_time) < DATA_COLLECTION_DURATION:
        height_tank_2, height_tank_1 = hf.read_from_serial(ser)

        if height_tank_2 is not None:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Calculate the new PWM value
            new_pwm_value = Controller.update(height_tank_2, TIME_STEP)
            new_pwm_value = max(1, min(new_pwm_value, 255)) # Ensure PWM stays within 1-255 range

            # Create a new DataFrame row for the current record, including the setpoint and the new PWM value
            new_data = pd.DataFrame([{
                'Timestamp': timestamp,
                'Setpoint': Controller.setpoint, # Include the setpoint here
                'PWM Value': new_pwm_value, # Use the new PWM value here
                'Height Tank 1': height_tank_1,
                'Height Tank 2': height_tank_2
            }])

            # Concatenate the new data with the existing DataFrame
            data = pd.concat([data, new_data], ignore_index=True)

            # Print statement now includes the setpoint and the new PWM value
            print(f"Recorded: {timestamp} - Setpoint: {Controller.setpoint}, PWM: {new_pwm_value}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2}")

            # Send the new PWM value to the Arduino
            hf.send_pwm_value(ser, new_pwm_value)

            # Wait for next evaluation
            time.sleep(TIME_STEP)

            # Reset height_tank_2 to None for the next read cycle, ensuring a fresh read
            height_tank_2 = None
        else:
            print("Failed to read valid data from Arduino. Retrying...")
            time.sleep(1) # Short delay before retrying read

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    ser.close()  # Close the serial port
    # Ensure the directory exists or handle the error
    try:
        data.to_csv('pid_data.csv', index=False)  # Save the DataFrame to a CSV file
        print(f"Data collected for {DATA_COLLECTION_DURATION} seconds (or until interrupted) and saved to 'pid_data.csv'.")
    except Exception as e:
        print(f"Error saving data to CSV: {e}")