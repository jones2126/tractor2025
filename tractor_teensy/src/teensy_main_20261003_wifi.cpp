/*********************************************************************
  teensy_main_20261003_wifi.cpp
  ---------------------------------
  Wi-Fi-primary low-level control for tractor01.

  This build leaves the proven 2026-09-26 drive calibration, steering PID,
  steering response watchdog, JRK diagnostics, physical E-stop relay handling,
  and telemetry intact. A valid WIFI serial feed can select Pause, Manual, or
  Auto without an NRF link.

  Command loss is enforced on the Teensy by the existing 500 ms watchdog:
      - transmission returns to exact neutral
      - steering output stops and holds its current position
      - a resumed stream automatically restores the requested phone state

  WIFI,<drive_percent>,<normalized_steering>,<mode>
      mode 0 = Pause, 1 = Manual, 2 = Auto
*********************************************************************/

#define TRACTOR_FIRMWARE_ID "teensy_main_20261003_wifi_v2"
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
    // parseSerialCommand() timestamps accepted input with a fresh millis().
    // Refresh the loop clock so unsigned age arithmetic cannot briefly see a
    // just-arrived command as older than the current control iteration.
    currentMillis = millis();
    if (wifiPhoneMessageCount > 0) {
        wifiPrimarySelected = true;
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
        const bool phoneFresh =
            currentMillis - wifiPhoneTimestamp <= CMD_VEL_TIMEOUT;

        if (!phoneFresh || wifiPhoneMode == 0) {
            // Explicit Pause and phone-heartbeat expiry share the proven
            // Pause behavior: exact transmission neutral, steering hold.
            radioData.control_mode = 2;
            wifiDriveCommand = true;
        } else if (wifiPhoneMode == 1) {
            // Manual always restores the phone's complete state after parsing,
            // so a competing CMD packet cannot seize control between beats.
            radioData.control_mode = 0;
            wifiDriveCommand = true;
            wifiDrivePercent = wifiManualDrivePercent;
            cmdVel.linear_x = 0.0f;
            cmdVel.angular_z = wifiSteeringCommand;
            cmdVel.timestamp = wifiPhoneTimestamp;
            cmdVel.received = true;
        } else {
            // Auto authorizes the independent navigation CMD stream. A fresh
            // phone heartbeat remains mandatory in addition to CMD freshness.
            radioData.control_mode = 0;
            wifiDriveCommand = false;
        }

        // Retain the fault recovery interlock without requiring NRF. Recovery
        // is permitted only after phone Pause and with neutral drive demand.
        if (wifiPhoneMode == 1 && phoneFresh && steeringFaultLatched &&
            steeringRecoveryPauseSeen && cmdVel.received &&
            fabsf(wifiManualDrivePercent) < 0.5f) {
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
