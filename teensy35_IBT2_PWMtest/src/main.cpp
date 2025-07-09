#include <Arduino.h>
#include <Teensy_PWM.h>

// Define pins
const int STEER_POT_PIN = A9;  // Potentiometer connected to A9
const int RPWM_PIN = 5;        // Right PWM pin connected to IBT-2 pin 1 (RPWM)
const int LPWM_PIN = 6;        // Left PWM pin connected to IBT-2 pin 2 (LPWM)
const int DEADBAND = 10;       // Deadband range (±10 around center 512)

// Instantiate Teensy_PWM objects with initial duty cycle of 0.0
Teensy_PWM PWM_R(RPWM_PIN, 15000, 0.0);  // 15 kHz, 0% duty for RPWM
Teensy_PWM PWM_L(LPWM_PIN, 15000, 0.0);  // 15 kHz, 0% duty for LPWM

// Timing for serial output (2 Hz = 500 ms)
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500;  // 500 ms for 2 Hz

void setup() {
  // Delay 60 seconds for RPi serial connection
  delay(60000);

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize pins as outputs
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  // Initialize PWM with 15 kHz and 0% duty cycle
  PWM_R.setPWM(RPWM_PIN, 15000, 0.0);
  PWM_L.setPWM(LPWM_PIN, 15000, 0.0);
}

void loop() {
  // Read potentiometer value
  int potValue = analogRead(STEER_POT_PIN);
  
  // Map potentiometer to motor speed (0-255)
  int motorSpeed = map(abs(potValue - 512), 0, 512, 0, 255);
  
  // Convert to percentage for Teensy_PWM (0.0 to 100.0)
  float dutyCycle = (motorSpeed / 255.0) * 100.0;
  
  // Check for deadband
  if (abs(potValue - 512) < DEADBAND) {
    // Stop motor
    PWM_R.setPWM(RPWM_PIN, 15000, 0.0);
    PWM_L.setPWM(LPWM_PIN, 15000, 0.0);
  } else if (potValue < 512) {
    // Forward direction
    PWM_R.setPWM(RPWM_PIN, 15000, dutyCycle);  // Forward
    PWM_L.setPWM(LPWM_PIN, 15000, 0.0);        // LPWM low
  } else {
    // Reverse direction
    PWM_L.setPWM(LPWM_PIN, 15000, dutyCycle);  // Reverse
    PWM_R.setPWM(RPWM_PIN, 15000, 0.0);        // RPWM low
  }

  // Print pot value and duty cycle at 2 Hz
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print("Pot Value: ");
    Serial.print(potValue);
    Serial.print(", Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.println("%");
    lastPrintTime = currentTime;
  }

  delay(10); // Small delay for stability
}