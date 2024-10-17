# The purpose of this code is to test the overall lap setup and verify you can read
# and write values from the Arduino. In this case, the default PWM signal is 128
# (about half open) and this makes sure it can be overwritten (using full open signal
# of 255).  Then, it checks that this value will hold with the arduino using the while
# loop that doesn't set the PWM signal.  It also verifies that it can read the anaolg
# signal coming from the tank all throughout.

import serial # pyserial
import time
import helper_functions as hf

# Set up the serial connection
ser = serial.Serial('COM3', 9600)  # Open serial port on appropriate COM port at 9600 baud
sleep_time = 1 # second

# Initial test: Send maximum PWM value 10 times
for i in range(10):
    hf.send_pwm_value(ser, 255)  # Send maximum PWM value (255) for full open valve
    print(hf.read_from_serial(ser))  # Read and print the response
    time.sleep(sleep_time)  # Wait for 1 second

print('Finished Sending PWM signal. Should hold going forward.')

# Continuous monitoring loop
while True:
    print(hf.read_from_serial(ser))  # Continuously read and print serial data
    time.sleep(sleep_time)  # Wait for 1 second between readings