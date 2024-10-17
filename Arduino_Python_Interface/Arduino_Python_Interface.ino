// Define constants and global variables
const int pwmPin = 12;  // PWM output pin (can be any PWM-capable pin)
int pwmValue = 128;  // Global variable to store the initial PWM value (0-255)

void setup() {
  // Initialize serial communication and pin modes
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output
  analogWrite(pwmPin, pwmValue);  // Set initial PWM value on the output pin
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    // Read the integer value sent from Python
  int newPwmValue = Serial.parseInt();
  
  // Update global pwmValue, constraining it to the valid PWM range (0-255)
  pwmValue = constrain(newPwmValue, 0, 255);
  
  // Set the new PWM output value if it's greater than 0
  // The reasoning for this is because even with the check for data available
  // on the serial port, it will sometimes read 0 if no data is there.  Because the CV
  // doesn't expel water until a value of 100, this isn't a problem for allowing
  // a full closed CV for the signal and should it read 0, it just holds the last value
  // by not updating.
  if (pwmValue > 0){
    analogWrite(pwmPin, pwmValue);
    }
  }
  
  // Read the analog input value from pin A0 (0-1023)
  // Note that which is the height of Tank 1 and which
  // is Tank 2 is dependent on the wire configureation
  int analogValue1 = analogRead(A0);
  int analogValue2 = analogRead(A1);
  
  // Send data back to Python via serial
  Serial.print(analogValue1);  // Send the current PWM value
  Serial.print(",");       // Separator between values
  Serial.println(analogValue2);  // Send the analog value and add a newline
  
  // Small delay to avoid flooding the serial port and allow for stable readings
  delay(100);  // 100 milliseconds delay
}