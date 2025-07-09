#include <Arduino.h>
#include <Teensy_PWM.h>

// Define pins
const int STEER_POT_PIN = A9;  // Potentiometer connected to A9
const int RPWM_PIN = 5;        // Right PWM pin connected to IBT-2 pin 1 (RPWM)
const int LPWM_PIN = 6;        // Left PWM pin connected to IBT-2 pin 2 (LPWM)
const int DEADBAND = 10;       // Deadband range (±10 around center 512)

// Instantiate Teensy_PWM objects for each pin
Teensy_PWM PWM_R(RPWM_PIN, 15000);  // 15 kHz for RPWM
Teensy_PWM PWM_L(LPWM_PIN, 15000);  // 15 kHz for LPWM

void setup() {
  // Initialize pins as outputs
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  // Initialize PWM
  PWM_R.begin();
  PWM_L.begin();
}

void loop() {
  // Read potentiometer value
  int potValue = analogRead(STEER_POT_PIN);
  
  // Map potentiometer to motor speed (0-255)
  int motorSpeed = map(abs(potValue - 512), 0, 512, 0, 255);
  
  // Check for deadband
  if (abs(potValue - 512) < DEADBAND) {
    // Stop motor
    PWM_R.setDutyCycle(0);
    PWM_L.setDutyCycle(0);
  } else if (potValue < 512) {
    // Forward direction
    PWM_R.setDutyCycle(motorSpeed / 255.0);  // Forward
    PWM_L.setDutyCycle(0);                   // LPWM low
  } else {
    // Reverse direction
    PWM_L.setDutyCycle(motorSpeed / 255.0);  // Reverse
    PWM_R.setDutyCycle(0);                   // RPWM low
  }

  delay(10); // Small delay for stability
}