#include <Arduino.h>
/*
IBT-2 steady-state test for Teensy 4.1
Drives RPWM pin 5 at PWM=150 continuously.
Measure voltage at IBT-2 motor output terminals with a multimeter.
Expected: ~7-8V at motor terminals (150/255 * 13.9V).
*/

int RPWM_Output = 5;
int LPWM_Output = 6;

void setup()
{
  Serial.begin(115200);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  Serial.println("IBT-2 steady test — RPWM pin 5 at PWM 150");
  Serial.println("Measure voltage at motor output terminals.");
}

void loop()
{
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, 150);
  Serial.println("RPWM=150 running...");
  delay(5000);
}
