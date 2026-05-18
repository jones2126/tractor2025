#include <Arduino.h>
/*
IBT-2 steady-state test for Teensy 4.1
Drives RPWM pin 5 at PWM=130 continuously (move RIGHT).
Reads and prints steering pot (A9) every second.
Pull fuse to reset IBT-2 overcurrent latch, then reinstall.
*/

#define RPWM_Output 5
#define LPWM_Output 6
#define STEER_POT_PIN A9

void setup()
{
  Serial.begin(115200);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  Serial.println("IBT-2 test: RPWM=130 (RIGHT), reading pot on A9");
}

void loop()
{
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, 130);

  int pot = analogRead(STEER_POT_PIN);
  Serial.print("RPWM=130  pot=");
  Serial.println(pot);
  delay(1000);
}
