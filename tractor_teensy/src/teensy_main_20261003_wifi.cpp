/*********************************************************************
  teensy_main_20261003_wifi.cpp
  ---------------------------------
  Wi-Fi-primary low-level control for tractor01.

  This build leaves the proven 2026-09-26 drive calibration, steering PID,
  steering response watchdog, JRK diagnostics, physical E-stop relay handling,
  and telemetry intact. A valid WIFI serial feed can select Pause or Manual
  without an NRF link.

  Command loss is enforced on the Teensy by the existing 500 ms watchdog:
      - transmission returns to exact neutral
      - steering output stops and holds its current position
      - a resumed stream automatically restores the requested phone state

  WIFI,<drive_percent>,<normalized_steering>,<manual>
      manual 0 = Pause, 1 = Manual
*********************************************************************/

#define TRACTOR_FIRMWARE_ID "teensy_main_20261003_wifi"
#define TRACTOR_TOP_LEVEL_LOOP loop_20260914_core
#include "teensy_main_20260914.cpp"


// Used only for the established Pause -> neutral Manual steering-fault
// recovery sequence. Normal Wi-Fi steering uses normalized Auto/PID control.
int normalizedSteerToRadio(float normalized) {
    normalized = constrain(normalized, -1.0f, 1.0f);
    if (normalized >= 0.0f) {
        return RADIO_STEER_CENTER + (int)(
            normalized * (RADIO_STEER_LEFT - RADIO_STEER_CENTER) + 0.5f
        );
    }
    return RADIO_STEER_CENTER + (int)(
        normalized * (RADIO_STEER_CENTER - RADIO_STEER_RIGHT) - 0.5f
    );
}


extern "C" void loop() {
    static bool wifiPrimarySelected = false;
    currentMillis = millis();

    // Preserve the existing physical E-stop relay path exactly. This runs
    // before any temporary Wi-Fi authority values are applied.
    estopCheck();

    parseSerialCommand();
    if (wifiDriveCommand && lastMotionCommandWasWifi) {
        wifiPrimarySelected = true;
    }
    if (wifiPrimarySelected) {
        // Once selected, a competing autonomous CMD publisher may pause the
        // machine but cannot seize steering or transmission. The next phone
        // heartbeat restores its complete Wi-Fi state.
        if (!lastMotionCommandWasWifi) cmdVel.received = false;
        wifiDriveCommand = true;
    }
    checkCmdVelTimeout();
    monitorSerialBuffer();
    publishPeriodicSystemIdentity();
    handleRadio();

    const bool savedSignalGood = radioStats.signalGood;
    const byte savedMode = radioData.control_mode;
    const int16_t savedSteering = radioData.steering_val;
    const int16_t savedTransmission = radioData.transmission_val;

    if (wifiPrimarySelected) {
        radioStats.signalGood = true;
        radioData.control_mode = wifiPhoneManual ? 0 : 2;

        // Retain the fault recovery interlock without requiring NRF. Recovery
        // is permitted only after phone Pause and with neutral drive demand.
        if (wifiPhoneManual && steeringFaultLatched &&
            steeringRecoveryPauseSeen && cmdVel.received &&
            fabsf(wifiDrivePercent) < 0.5f) {
            radioData.control_mode = 1;
            radioData.transmission_val = 600;  // bucket 4: exact neutral
            radioData.steering_val = normalizedSteerToRadio(cmdVel.angular_z);
        }
    }

    controlTransmission();
    controlSteering();

    // Keep separately published NRF diagnostics truthful.
    radioStats.signalGood = savedSignalGood;
    radioData.control_mode = savedMode;
    radioData.steering_val = savedSteering;
    radioData.transmission_val = savedTransmission;

    debugSteerPot();
}
