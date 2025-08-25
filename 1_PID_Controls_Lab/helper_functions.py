import serial # pyserial
import time
import pandas as pd
import numpy as np
from datetime import datetime
from scipy.optimize import minimize

def read_from_serial(ser):
    """
    Reads data from the serial port and parses it into PWM and analog values.
    
    Args:
    ser (serial.Serial): An open serial port object.
    
    Returns:
    tuple: A tuple containing (pwm_value, analog_value), or (None, None) if no valid data is available.
    
    This function does the following:
    1. Checks if there's data available to read from the serial port.
    2. If data is available, it reads a line, decodes it from bytes to a UTF-8 string, and removes any whitespace.
    3. Splits the line into two parts using a comma as the separator.
    4. If there are exactly two parts, it attempts to convert them to integers.
    5. If successful, it returns the two values as a tuple.
    6. If any step fails (no data, wrong format, can't convert to int), it returns (None, None).
    """
    # Flush the input buffer to discard old data
    ser.reset_input_buffer()

    while ser.in_waiting <= 0:
        time.sleep(0.1)
    if ser.in_waiting > 0:
        analog_value_1 = None
        analog_value_2 = None
        while analog_value_1 == None and analog_value_2 == None:
            line = ser.readline().decode('utf-8').strip()  # Read and decode the line
            parts = line.split(',')  # Split the line into PWM and analog values
            if len(parts) == 2:
                try:
                    analog_value_1 = int(parts[0])
                    analog_value_2 = int(parts[1])
                except ValueError:
                    pass
        return analog_value_1, analog_value_2
    return None, None

def send_pwm_value(ser, value, sleep_time = 0):
    """
    Sends a PWM value to the Arduino through the serial port.
    
    Args:
    ser (serial.Serial): An open serial port object.
    value (int): The PWM value to send. Should be between 0 and 255, inclusive.
    
    This function does the following:
    1. Checks if the provided value is within the valid PWM range (0-255).
    2. If valid, it converts the value to a string, adds a newline character, 
       encodes it to bytes, and sends it over the serial connection.
    3. Waits for a short period (0.1 seconds) to ensure the Arduino has time to process the sent value.
    
    Note: This function does not return anything or provide feedback if the value is out of range.
    It silently fails if the value is invalid.
    """
    # Clip the value between 1 and 255
    value = max(1, min(value, 255))
    ser.flushInput() # This is the critical function to ensure the most up to date data is recieved and input
    ser.write(f"{value}\n".encode()) # Send the PWM value followed by a newline character
    ser.flush()
    time.sleep(sleep_time)  # Small delay to ensure Arduino has time to process

class MPC:
    def __init__(self, model_tank1, model_tank2, Q2, R, Np, Nc, x0, time_step):
        """
        Initialize the MPC object.
        
        Args:
        model_tank1 (callable): Trained model to predict Tank 1 dynamics.
        model_tank2 (callable): Trained model to predict Tank 2 dynamics.
        Q2 (np.ndarray): State cost matrix for Tank 2 (h2).
        R (np.ndarray): Input cost matrix.
        Np (int): Prediction horizon.
        Nc (int): Control horizon.
        x0 (np.ndarray): Initial state (heights of both tanks).
        """
        self.model_tank1 = model_tank1
        self.model_tank2 = model_tank2
        self.Q2 = Q2  # Cost only for h2 (Tank 2)
        self.R = R
        self.Np = Np  # Prediction horizon
        self.Nc = Nc  # Control horizon
        self.x0 = x0  # Initial state
        self.time_step = time_step

    def predict_state(self, x, u_sequence):
        """
        Predict the future state trajectory based on the current state and a sequence of control inputs.
        
        Args:
        x (np.ndarray): Current state (heights of both tanks).
        u_sequence (np.ndarray): Control input sequence.

        Returns:
        np.ndarray: Predicted state trajectory.
        """
        n = len(x)
        state_trajectory = np.zeros((self.Np + 1, n))
        state_trajectory[0] = x
        
        h1, h2 = x[0], x[1]
        
        for i in range(self.Np):
            u = u_sequence[i] if i < len(u_sequence) else u_sequence[-1]
            
            # Use the trained models to predict the next heights
            h1_next = h1 + self.time_step*self.model_tank1.predict(pd.DataFrame({'PWM':u,'Tank 1':h1,'Tank 2':h2}).to_numpy())  # Model prediction for Tank 1
            h2_next = h2 + self.time_step*self.model_tank2.predict(pd.DataFrame({'PWM':u,'Tank 1':h1,'Tank 2':h2}).to_numpy())  # Model prediction for Tank 2
            state_trajectory[i + 1] = [h1_next.item(), h2_next.item()]
            h1, h2 = h1_next, h2_next  # Update for the next prediction
        
        return state_trajectory

    def cost_function(self, u_sequence, x, x_ref):
        """
        The cost function to minimize, where we only penalize deviations of Tank 2 (h2).
        
        Args:
        u_sequence (np.ndarray): Control input sequence (flattened).
        x (np.ndarray): Current state of the system.
        x_ref (np.ndarray): Reference trajectory for Tank 2's height (h2).

        Returns:
        float: The cost for the given control input sequence.
        """
        # Reshape the control input sequence back into 2D form
        u_sequence = u_sequence.reshape(self.Nc, 1)
        
        state_trajectory = self.predict_state(x, u_sequence)
        cost = 0

        for i in range(self.Np):
            h2_diff = state_trajectory[i][1] - x_ref[1]  # Only consider Tank 2's height deviation
            u = u_sequence[i] if i < len(u_sequence) else u_sequence[-1]
            # cost += h2_diff.T @ self.Q2 @ h2_diff + u.T @ self.R @ u
            cost += h2_diff**2 #+ u.T @ self.R @ u

        return cost

    def solve(self, x, x_ref):
        """
        Solve the MPC optimization problem using scipy.optimize.minimize.
        
        Args:
        x (np.ndarray): Current state of the system.
        x_ref (np.ndarray): Reference trajectory for Tank 2's height.

        Returns:
        np.ndarray: Optimal control input sequence.
        """
        # Initial guess for the control input (a flat input sequence)
        u_initial = np.zeros(self.Nc)

        # Minimize the cost function using SciPy
        result = minimize(
            self.cost_function, 
            u_initial, 
            args=(x, x_ref), 
            method='SLSQP', 
            bounds=[(0, 255)] * self.Nc # Bound Input Space
        )

        if not result.success:
            raise ValueError("MPC optimization problem did not converge")

        # Return the first control input
        u_optimal = result.x
        return u_optimal[0]

    def apply_control(self, x, x_ref):
        """
        Apply control by solving the MPC and updating the state.
        
        Args:
        x (np.ndarray): Current state of the system.
        x_ref (np.ndarray): Reference trajectory for Tank 2's height.
        
        Returns:
        np.ndarray: New state after applying the control.
        """
        u_opt = self.solve(x, x_ref)
        # h1_next = self.model_tank1.predict([x[0], x[1], u_opt])[0]
        # h2_next = self.model_tank2.predict([h1_next, x[1]])[0]
        # x_next = np.array([h1_next, h2_next])
        return u_opt # x_next, u_opt
    
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        """
        Initialize the PID controller.
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            setpoint (float): Desired value of the controlled variable
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0  # Accumulated integral error
        self.previous_error = 0  # Error from the previous update
    
    def update(self, measured_value, dt):
        """
        Update the PID controller and calculate the control output.
        
        Args:
            measured_value (float): Current value of the controlled variable
            dt (float): Time step since the last update
        
        Returns:
            float: The control output
        """
        # Calculate error (difference between setpoint and measured value)
        error = self.setpoint - measured_value
        
        # Proportional term: proportional to the current error
        P = self.Kp * error
        
        # Integral term: sum of all past errors
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term: rate of change of error
        derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative
        
        # Compute the control output by summing P, I, and D terms
        output = P + I + D
        
        # Store the current error for the next iteration
        self.previous_error = error
        
        return output

def ziegler_nichols_tuning(ser, setpoint=1, initial_Kp=1, max_iterations=100, dt=1):
    """
    Perform Ziegler-Nichols tuning for a PID controller using real-time data from a serial connection.

    This function implements the Ziegler-Nichols tuning method to find optimal PID controller parameters.
    It gradually increases the proportional gain until the system begins to oscillate, then uses the 
    critical gain and oscillation period to calculate the PID parameters.

    Args:
        ser (serial.Serial): An open serial port object for communication with the process.
        setpoint (float): The desired value for the controlled variable. Default is 1.
        initial_Kp (float): The initial value for the proportional gain. Default is 1.
        max_iterations (int): The maximum number of iterations to run the tuning. Default is 100.
        dt (float): The time step for each iteration in seconds. Default is 1.

    Returns:
        tuple: A tuple containing:
            - Kp_final (float): The final proportional gain, or None if tuning failed.
            - Ki_final (float): The final integral gain, or None if tuning failed.
            - Kd_final (float): The final derivative gain, or None if tuning failed.
            - data (pd.DataFrame): A DataFrame containing recorded data during tuning.

    Notes:
        - The function communicates with a real process through the serial connection.
        - It increases Kp until sustained oscillations are detected.
        - PID parameters are calculated using Ziegler-Nichols formulas (classic PID):
          Kp = 0.6 * K_crit
          Ki = 2 * Kp / T_crit
          Kd = Kp * T_crit / 8
        - The process variable being controlled is the height of Tank 2.
    """
    # Assign preliminary values and data structures
    Kp = initial_Kp
    Ki = Kd = 0
    pid = PIDController(Kp, Ki, Kd, setpoint)
    measured_values = []
    oscillation_started = False
    K_crit = T_crit = None
    prev_sign = cross_time = None
    osc_count = 0
    pwm_value = 128  # Initial PWM value (half open control valve)
    data = pd.DataFrame(columns=['Timestamp', 'PWM Value', 'Height Tank 2', 'Kp'])
    
    for i in range(max_iterations):
        # Read the tuple and extract the first element for Tank 2
        height_tank_2_tuple = read_from_serial(ser)
        height_tank_2 = height_tank_2_tuple[0]  # Extract the first element for Tank 2
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        new_data = pd.DataFrame([{'Timestamp': timestamp, 'PWM Value': pwm_value, 'Height Tank 2': height_tank_2, 'Kp': Kp}])
        data = pd.concat([data, new_data], ignore_index=True)
        print(f"Recorded: {timestamp} - PWM: {pwm_value}, Height Tank 2: {height_tank_2}")

        pwm_value = pid.update(height_tank_2, dt) # Update PWM value based on PID output
        send_pwm_value(ser, pwm_value)
        measured_values.append(height_tank_2)

        time.sleep(dt)
                         
        # Check if Oscillation Started
        if i > 1 and not oscillation_started:
            if (measured_values[-1] - setpoint) * (measured_values[-2] - setpoint) < 0:
                oscillation_started = True
                prev_sign = np.sign(measured_values[-1] - setpoint)
                cross_time = i * dt
                osc_count = 1

        # Count Number of Oscillations
        if oscillation_started and (measured_values[-1] - setpoint) * prev_sign < 0:
            prev_sign = np.sign(measured_values[-1] - setpoint)
            osc_count += 1
            if osc_count == 3:
                T_crit = (i * dt) - cross_time
                K_crit = Kp
                break
        
        # Update Kp for next iteration
        Kp += 0.1
        pid.Kp = Kp

    # After completed tuning, provide results
    if K_crit and T_crit:
        print(f"Ziegler-Nichols tuning results: K Ultimate={K_crit}, T Ultimate={T_crit}")
        return K_crit, T_crit, data
    else:
        print("Oscillations did not start.")
        return None, None, data