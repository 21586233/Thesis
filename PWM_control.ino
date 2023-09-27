#include <Servo.h>

// Pin assignments
const int potPin1 = A15;          // Analog input for potentiometer 1
const int potPin2 = A14;          // Analog input for potentiometer 2
const int escControlPin1 = 2;    // Digital output for ESC 1 control signal
const int escControlPin2 = 3;   // Digital output for ESC 2 control signal

// ESC parameters
const int escMinPulse = 1000;    // Minimum pulse width for ESC (microseconds)
const int escMaxPulse = 2000;    // Maximum pulse width for ESC (microseconds)
const int escNeutralPulse = 1500; // Neutral pulse width for ESC (microseconds)

Servo esc1;                       // Servo object to control ESC 1
Servo esc2;                       // Servo object to control ESC 2

void setup() {
  Serial.begin(9600);
  esc1.attach(escControlPin1);    // Attach servo object for ESC 1
  esc2.attach(escControlPin2);    // Attach servo object for ESC 2
  
  esc1.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC 1
  esc2.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC 2
  
  delay(2000);                    // Wait for ESCs to initialize
}

void loop() {
  // Read potentiometer values
  int potValue1 = analogRead(potPin1);
  int potValue2 = analogRead(potPin2);

  // Map the potentiometer values to ESC control range
  int escPulseWidth1 = map(potValue1, 0, 300, escMinPulse, escMaxPulse);
  int escPulseWidth2 = map(potValue2, 0, 315, escMinPulse, escMaxPulse);

  // Send ESC control signals
  esc1.writeMicroseconds(escPulseWidth1);
  esc2.writeMicroseconds(escPulseWidth2);

  // Print potentiometer values and ESC pulse widths for debugging
  Serial.print("L: ");
  Serial.print(potValue1);
  Serial.print("\tESC 1 Pulse Width: ");
  Serial.println(escPulseWidth1);

  Serial.print("R: ");
  Serial.print(potValue2);
  Serial.print("\tESC 2 Pulse Width: ");
  Serial.println(escPulseWidth2);

  // Delay for stability
  delay(1000);
}
