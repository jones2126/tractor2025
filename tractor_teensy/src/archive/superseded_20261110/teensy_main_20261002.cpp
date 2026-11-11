/*********************************************************************
  teensy_main_20261002.cpp
  --------------------------------------------------------------
  Telemetry-only successor to teensy_main_20260926.

  Control behavior is inherited unchanged. This wrapper adds a 4 Hz
  RADIO_INPUTS serial record containing all four handheld values already
  received in the existing 14-byte NRF24 payload. No pin mapping, steering,
  transmission, radio, watchdog, or safety behavior is changed.
*********************************************************************/

#define TRACTOR_FIRMWARE_ID "teensy_main_20261002"
#define TRACTOR_TOP_LEVEL_LOOP loop_20260926
#include "teensy_main_20260914.cpp"
#undef TRACTOR_TOP_LEVEL_LOOP
#undef TRACTOR_FIRMWARE_ID


void publishCenteredInputTelemetry() {
    static unsigned long lastInputPrint = 0;
    const unsigned long inputPrintInterval = 250;
    if (currentMillis - lastInputPrint < inputPrintInterval) return;

    char buf[112];
    unsigned long radioAge = currentMillis - radioStats.lastAckTime;
    snprintf(
        buf,
        sizeof(buf),
        "1,%lu,RADIO_INPUTS,s=%d,t=%d,x=%d,p4=%d,sg=%d,a=%lu",
        currentMillis,
        (int)radioData.steering_val,
        (int)radioData.throttle_val,
        (int)radioData.transmission_val,
        (int)radioData.pot4_val,
        radioStats.signalGood ? 1 : 0,
        radioAge
    );
    Serial.println(buf);
    lastInputPrint = currentMillis;
}


extern "C" void loop() {
    loop_20260926();
    publishCenteredInputTelemetry();
}
