#include <Arduino.h>
#include <Teensy_PWM.h>

// Define potentiometer pin
const int STEER_POT_PIN = A9;  // Potentiometer connected to A9
const int RPWM_Output = 5;     // Right PWM pin connected to IBT-2 pin 1 (RPWM)
const int LPWM_Output = 6;     // Left PWM pin connected to IBT-2 pin 2 (LPWM)
const int DEADBAND = 10;       // Deadband range (±10 around center 512)

void setup() {
  // Initialize the motor control pins as output
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  // Set PWM frequency to 15 kHz
  PWM.begin(RPWM_Output, 15000); // Pin 5 with 15 kHz PWM frequency
  PWM.begin(LPWM_Output, 15000); // Pin 6 with 15 kHz PWM frequency
}

void loop() {
  // Read the value from the potentiometer (0 to 1023)
  int potValue = analogRead(STEER_POT_PIN);
  
  // Map the potentiometer value to a speed range (0 to 255)
  int motorSpeed = map(abs(potValue - 512), 0, 512, 0, 255);
  
  // Check if potentiometer is within deadband
  if (abs(potValue - 512) < DEADBAND) {
    // Stop motor
    PWM.setDutyCycle(RPWM_Output, 0);
    PWM.setDutyCycle(LPWM_Output, 0);
  } else if (potValue < 512) {
    // Forward direction
    PWM.setDutyCycle(RPWM_Output, motorSpeed / 255.0);  // Forward with mapped speed
    PWM.setDutyCycle(LPWM_Output, 0);                    // LPWM is low for forward
  } else {
    // Reverse direction
    PWM.setDutyCycle(LPWM_Output, motorSpeed / 255.0);  // Reverse with mapped speed
    PWM.setDutyCycle(RPWM_Output, 0);                    // RPWM is low for reverse
  }

  delay(10); // Small delay for stability
}