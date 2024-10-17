import serial # pyserial
import helper_functions as hf

# Set up the serial connection
ser = serial.Serial('COM3', 9600)  # Adjust the port as needed

# Uses function to output PID tuning parameters and data from tuning
hf.ziegler_nichols_tuning(ser, 
                          setpoint=350, 
                          initial_Kp=5, 
                          max_iterations=250, 
                          dt=1)