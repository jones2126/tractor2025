#include <Arduino.h>
/*
Simple test for IBT-2 H-Bridge with Teensy 3.5
Runs motor forward and reverse in a timed loop.
*/

int RPWM_Output = 5; // Connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Connect to IBT-2 pin 2 (LPWM)

void setup()
{
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
}

void loop()
{
  // Forward at 50% speed
  analogWrite(RPWM_Output, 0);     // Turn off reverse
  analogWrite(LPWM_Output, 128);   // Set forward speed (0-255)
  delay(3000);                     // Run forward for 3 seconds

  // Stop
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  delay(1000);                     // Pause for 1 second

  // Reverse at 50% speed
  analogWrite(LPWM_Output, 0);     // Turn off forward
  analogWrite(RPWM_Output, 128);   // Set reverse speed
  delay(3000);                     // Run reverse for 3 seconds

  // Stop
  analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, 0);
  delay(1000);                     // Pause before repeating
}
