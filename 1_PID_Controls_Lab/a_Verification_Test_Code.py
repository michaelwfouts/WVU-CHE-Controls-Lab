# The purpose of this code is to test the overall lap setup and verify you can read
# and write values from the Arduino. In this case, the default PWM signal is 255
# (full open).  Then, it checks that this value will hold with the arduino using the while
# loop that doesn't set the PWM signal.  It also verifies that it can read the anaolg
# signal coming from the tank all throughout.

import serial # pyserial
import time
import helper_functions as hf

# Set up the serial connection
ser = serial.Serial('COM3', 9600)  # Open serial port on appropriate COM port at 9600 baud
PWM_VALUE = 255  # Default PWM value (half open valve)
sleep_time = 1 # second

# Initial test: Send maximum PWM value 10 times
for i in range(10):
    height_tank_2, height_tank_1 = hf.read_from_serial(ser)
    hf.send_pwm_value(ser, PWM_VALUE)  # Send maximum PWM value (255) for full open valve
    print(f"PWM: {PWM_VALUE}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2})") # Print the control valve signal and heights
    time.sleep(sleep_time)  # Wait for 1 second

print('Finished Sending PWM signal. Will hold going forward.')

# Continuous monitoring loop
while True:
    height_tank_2, height_tank_1 = hf.read_from_serial(ser) # Read heights from tanks
    print(f"Held PWM: {PWM_VALUE}, Height Tank 1: {height_tank_1}, Height Tank 2: {height_tank_2})") # Print the control valve signal and heights
    time.sleep(sleep_time)  # Wait for 1 second between readings