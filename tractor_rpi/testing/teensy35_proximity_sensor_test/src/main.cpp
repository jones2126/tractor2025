/*
 * Interrupt-Based Proximity Sensor Test Program for Teensy 3.5
 * 
 * This program uses interrupts to detect wheel movement using two 
 * URBEST 8mm inductive proximity sensors connected to pins 25 and 26.
 * 
 * The sensors output:
 * - HIGH when NO metal detected
 * - LOW when metal is detected (triggers interrupt on FALLING edge)
 * 
 * Hardware:
 * - Left wheel sensor: Pin 25
 * - Right wheel sensor: Pin 26
 * - Built-in LED: Pin 13
 */
#include <Arduino.h>
// Pin definitions
const int LEFT_SENSOR_PIN = 25;
const int RIGHT_SENSOR_PIN = 26;
const int LED_PIN = 13;

// Volatile variables for interrupt counters
volatile unsigned long leftTicks = 0;   // Left wheel encoder ticks
volatile unsigned long rightTicks = 0;  // Right wheel encoder ticks

// Variables for movement detection
volatile bool leftMovement = false;
volatile bool rightMovement = false;

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // Print every 1000ms

// Previous tick counts for calculating speed
unsigned long prevLeftTicks = 0;
unsigned long prevRightTicks = 0;

// Interrupt Service Routines (ISRs)
void leftSensorISR() {
  leftTicks++;
  leftMovement = true;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED on left sensor
}

void rightSensorISR() {
  rightTicks++;
  rightMovement = true;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED on right sensor
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);   // Enable internal pullup
  pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);  // Enable internal pullup
  pinMode(LED_PIN, OUTPUT);
  
  // Attach interrupts to detect FALLING edge (metal detection)
  // FALLING edge occurs when metal approaches sensor (HIGH to LOW transition)
  attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), leftSensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), rightSensorISR, FALLING);
  
  // Wait for serial monitor to open
  delay(2000);
  
  Serial.println("=== Interrupt-Based Proximity Sensor Test ===");
  Serial.println("Left sensor on pin 25, Right sensor on pin 26");
  Serial.println("Interrupts trigger on FALLING edge (metal detection)");
  Serial.println("LED toggles on each detection");
  Serial.println("Format: Left Ticks | Right Ticks | Left RPM | Right RPM");
  Serial.println();
  
  digitalWrite(LED_PIN, LOW); // Start with LED off
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for movement flags set by interrupts
  if (leftMovement) {
    Serial.print("L"); // Quick indication of left movement
    leftMovement = false;
  }
  
  if (rightMovement) {
    Serial.print("R"); // Quick indication of right movement  
    rightMovement = false;
  }
  
  // Periodic detailed status report
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    // Calculate ticks per second (movement rate)
    unsigned long leftDelta = leftTicks - prevLeftTicks;
    unsigned long rightDelta = rightTicks - prevRightTicks;
    
    // Calculate approximate RPM if you know ticks per revolution
    // For now, just showing ticks per second
    float leftTicksPerSec = leftDelta / (PRINT_INTERVAL / 1000.0);
    float rightTicksPerSec = rightDelta / (PRINT_INTERVAL / 1000.0);
    
    Serial.println(); // New line after any L/R indicators
    Serial.print("Ticks - Left: ");
    Serial.print(leftTicks);
    Serial.print(" (+");
    Serial.print(leftDelta);
    Serial.print("), Right: ");
    Serial.print(rightTicks);
    Serial.print(" (+");
    Serial.print(rightDelta);
    Serial.print(")");
    
    Serial.print(" | Rate - Left: ");
    Serial.print(leftTicksPerSec, 1);
    Serial.print(" ticks/sec, Right: ");
    Serial.print(rightTicksPerSec, 1);
    Serial.println(" ticks/sec");
    
    // Store current counts for next calculation
    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;
    lastPrintTime = currentTime;
  }
  
  // Small delay to prevent overwhelming serial output
  delay(10);
}