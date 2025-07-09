#include <Arduino.h>

// Define pins
const int STEER_POT_PIN = A9;  // Potentiometer connected to A9
const int RPWM_PIN = 5;        // Right PWM pin connected to IBT-2 pin 1 (RPWM)
const int LPWM_PIN = 6;        // Left PWM pin connected to IBT-2 pin 2 (LPWM)
const int DEADBAND = 10;       // Deadband range (±10 around center 512)

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

  // Set PWM frequency to 15 kHz for pins 5 and 6 (FTM0)
  analogWriteFrequency(5, 15000);

  // Initialize PWM to 0
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void loop() {
  // Read potentiometer value with averaging
  int total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(STEER_POT_PIN);
    delay(1);
  }
  int potValue = total / 5;
  
  // Map potentiometer to motor speed (0-255)
  int motorSpeed = map(abs(potValue - 512), 0, 512, 0, 255);
  
  // Cap duty cycle at 95% (242/255)
  motorSpeed = min(motorSpeed, 242);
  
  // Calculate duty cycle percentage for serial output
  float dutyCycle = (motorSpeed / 255.0) * 100.0;
  
  // Motor direction for serial output
  String direction;
  
  // Check for deadband
  if (abs(potValue - 512) < DEADBAND) {
    // Stop motor
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    direction = "Stop";
  } else if (potValue < 512) {
    // Reverse direction (potValue = 0 gives 95% duty)
    analogWrite(LPWM_PIN, motorSpeed);  // Reverse
    analogWrite(RPWM_PIN, 0);          // RPWM low
    direction = "Reverse";
  } else {
    // Forward direction (potValue = 1023 gives 95% duty)
    analogWrite(RPWM_PIN, motorSpeed);  // Forward
    analogWrite(LPWM_PIN, 0);          // LPWM low
    direction = "Forward";
  }

  // Print pot value, duty cycle, and direction at 2 Hz
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print("Pot Value: ");
    Serial.print(potValue);
    Serial.print(", Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.print("%, Direction: ");
    Serial.println(direction);
    lastPrintTime = currentTime;
  }

  delay(10); // Small delay for stability
}