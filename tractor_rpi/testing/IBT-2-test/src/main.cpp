#include <Arduino.h>
/*
IBT-2 H-Bridge test for Teensy 4.1
Wheel starts hard left — first movement drives RPWM (expected: move RIGHT).
Watch the wheel direction and note which pin moves which way.
PWM = 150 (above stall threshold).
*/

int RPWM_Output = 5; // Connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Connect to IBT-2 pin 2 (LPWM)

void setup()
{
  Serial.begin(115200);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  Serial.println("IBT-2 test starting in 3 seconds...");
  delay(3000);
}

void loop()
{
  // Step 1: Drive RPWM — wheel should move RIGHT (away from hard left)
  Serial.println("RPWM ON (pin 5) — wheel should move RIGHT");
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, 150);
  delay(3000);

  // Stop
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  Serial.println("STOP");
  delay(2000);

  // Step 2: Drive LPWM — wheel should move LEFT
  Serial.println("LPWM ON (pin 6) — wheel should move LEFT");
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 150);
  delay(3000);

  // Stop
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  Serial.println("STOP");
  delay(2000);
}
