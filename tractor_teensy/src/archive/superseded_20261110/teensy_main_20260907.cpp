/*********************************************************************
  teensy_main_20260907.cpp
  --------------------------------------------------------------
  2026-09-07 speed-calibration test firmware.

  This source intentionally reuses the archived, field-tested
  teensy_main_20260804.cpp implementation and replaces only:
    - Auto m/s -> JRK calibration
    - controlTransmission() so its telemetry reports the new target
    - setup() firmware identity
    - loop() so it calls the 2026-09-07 transmission function

  Manual bucketTargets[] remains exactly as defined in the archived
  20260804 firmware:
      3138, 3063, 2987, 2912, 2836,
      2616, 2534, 2452, 2370, 2288

  Auto calibration for the Ring 13 test:
      1.00 m/s -> JRK 2288  known reference
      1.25 m/s -> JRK 2240  experimental
      1.50 m/s -> JRK 2200  experimental

  The established lower-speed points are retained, including
      0.94 m/s -> JRK 2404.

  Requires:
      src/archive/teensy_main_20260804.cpp
*********************************************************************/

// Rename only the functions that this 0907 file replaces.  Everything else
// (radio, steering watchdog, JRK diagnostics, E-stop, telemetry helpers, etc.)
// is compiled directly from the known-good archived 0804 source.
#define mpsToJrkTarget      mpsToJrkTarget_20260804
#define controlTransmission controlTransmission_20260804
#define setup               setup_20260804
#define loop                loop_20260804

#include "archive/teensy_main_20260804.cpp"

#undef loop
#undef setup
#undef controlTransmission
#undef mpsToJrkTarget


// -------------------------------------------------------------------
// 2026-09-07 Auto-mode m/s -> JRK target calibration.
//
// IMPORTANT:
//   * Lower JRK target = farther forward.
//   * 2288 is retained as the Manual-mode maximum because bucketTargets[]
//     comes unchanged from the archived 0804 firmware.
//   * 2240 and 2200 are experimental Auto-only extensions for Ring 13.
const SpeedCalPoint SPEED_CAL_20260907[] = {
    {0.00f, 2836},
    {0.40f, 2452},
    {0.87f, 2421},  // measured: 0.899 outbound, 0.846 return
    {0.94f, 2404},  // measured: 0.974 outbound, 0.898 return

    {1.00f, 2288},  // known Ring 13 reference: approximately 1.0 m/s actual
    {1.25f, 2240},  // experimental
    {1.50f, 2200},  // experimental
};

const int SPEED_CAL_20260907_POINTS =
    sizeof(SPEED_CAL_20260907) / sizeof(SPEED_CAL_20260907[0]);

uint16_t mpsToJrkTarget(float mps) {
    if (mps <= SPEED_CAL_20260907[0].mps)
        return SPEED_CAL_20260907[0].jrkTarget;

    if (mps >= SPEED_CAL_20260907[SPEED_CAL_20260907_POINTS - 1].mps)
        return SPEED_CAL_20260907[SPEED_CAL_20260907_POINTS - 1].jrkTarget;

    for (int i = 1; i < SPEED_CAL_20260907_POINTS; i++) {
        if (mps <= SPEED_CAL_20260907[i].mps) {
            float f =
                (mps - SPEED_CAL_20260907[i - 1].mps) /
                (SPEED_CAL_20260907[i].mps -
                 SPEED_CAL_20260907[i - 1].mps);

            float t =
                (float)SPEED_CAL_20260907[i - 1].jrkTarget +
                f * ((float)SPEED_CAL_20260907[i].jrkTarget -
                     (float)SPEED_CAL_20260907[i - 1].jrkTarget);

            return (uint16_t)(t + 0.5f);
        }
    }

    return SPEED_CAL_20260907[0].jrkTarget;  // defensive/unreachable
}


// -------------------------------------------------------------------
// Transmission control (10 Hz).
//
// This is the 0804 production logic with the Auto target calculation routed
// through the 2026-09-07 calibration above.  Keeping the complete function
// here also ensures TRANS/TL telemetry reports the ACTUAL 0907 requestedTarget.
void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun <
        controlTransmissionInterval) return;

    if (!radioStats.signalGood) {
        radioData.control_mode = 9;   // safety
    }

    uint16_t requestedTarget = transmissionNeutralPos;
    int bucketTmp = bucket;

    switch (radioData.control_mode) {
        case 0:
            // Auto mode: cmd_vel speed in m/s.
            if (cmdVel.received) {
                requestedTarget = mpsToJrkTarget(cmdVel.linear_x);
                bucketTmp = 5;
            } else {
                requestedTarget = transmissionNeutralPos;
                bucketTmp = 5;
            }
            break;

        case 1:
            // Manual bucket mode.  Uses the UNCHANGED 0804 bucketTargets[].
            {
                int tv = (int)radioData.transmission_val;

                if      (tv >= 931) bucketTmp = 0;
                else if (tv >= 838) bucketTmp = 1;
                else if (tv >= 746) bucketTmp = 2;
                else if (tv >= 654) bucketTmp = 3;
                else if (tv >= 562) bucketTmp = 4;
                else if (tv >= 469) bucketTmp = 5;
                else if (tv >= 377) bucketTmp = 6;
                else if (tv >= 285) bucketTmp = 7;
                else if (tv >= 192) bucketTmp = 8;
                else                bucketTmp = 9;

                requestedTarget = bucketTargets[bucketTmp];
            }
            break;

        case 2:
            // Pause mode.
            requestedTarget = transmissionNeutralPos;
            break;

        default:
            // Includes mode 9: radio-loss safety.
            requestedTarget = transmissionNeutralPos;
            break;
    }

    if (steeringFaultLatched) {
        if (radioData.control_mode == 2) {
            steeringRecoveryPauseSeen = true;
        }

        // A successful Manual steering response after Pause proves that the
        // actuator is available again. Require the transmission command to be
        // neutral before clearing so recovery cannot cause a surprise launch.
        const bool recoveryComplete =
            radioData.control_mode == 1 &&
            steeringRecoveryPauseSeen &&
            steeringManualResponseOK &&
            requestedTarget == transmissionNeutralPos;

        if (recoveryComplete) {
            clearSteeringFaultAfterManualRecovery();
        } else {
            requestedTarget = transmissionNeutralPos;
            bucketTmp = 4;
        }
    }

    bucket = bucketTmp;

    // Continue commanding the JRK at the normal transmission-control rate.
    setJrkTarget(requestedTarget);

    // Poll a coherent JRK diagnostic snapshot at 5 Hz.
    static unsigned long lastJrkFeedbackRead = 0;
    static const unsigned long jrkFeedbackInterval = 200;

    if (currentMillis - lastJrkFeedbackRead >= jrkFeedbackInterval) {
        updateJrkDiagnostics();
        currentTransmissionOutput = jrkDiagnostics.feedback;
        lastJrkFeedbackRead = currentMillis;
    }

    // Publish machine-readable telemetry at 5 Hz.
    static unsigned long lastTransStatusPrint = 0;
    static const unsigned long transStatusInterval = 200;

    if (currentMillis - lastTransStatusPrint >= transStatusInterval) {
        char buf[320];

        snprintf(
            buf,
            sizeof(buf),
            "1,%lu,TRANS,m=%d,b=%d,tgt=%u,cur=%u,at=%u,sfb=%u,"
            "it=%d,dtt=%d,dc=%d,eh=%u,eo=%u,jq=%lu,jv=%d,jl=%u,"
            "jto=%lu,jdb=%lu,rv=%d,x=%.3f,ca=%lu",
            currentMillis,
            radioData.control_mode,
            bucket,
            requestedTarget,
            currentTransmissionOutput,
            jrkDiagnostics.actualTarget,
            jrkDiagnostics.scaledFeedback,
            jrkDiagnostics.integral,
            jrkDiagnostics.dutyCycleTarget,
            jrkDiagnostics.dutyCycle,
            jrkDiagnostics.errorsHalting,
            jrkDiagnostics.errorsOccurred,
            (unsigned long)jrkDiagnostics.sequence,
            jrkDiagnostics.valid ? 1 : 0,
            jrkDiagnostics.readLatencyMs,
            (unsigned long)jrkDiagnostics.timeouts,
            (unsigned long)jrkDiagnostics.discardedBytes,
            (int)radioData.transmission_val,
            cmdVel.linear_x,
            cmdVel.received
                ? currentMillis - cmdVel.timestamp
                : 999999UL
        );

        Serial.println(buf);
        lastTransStatusPrint = currentMillis;
    }

    // Optional low-rate Auto-mode diagnostic; reuse cached JRK feedback.
    if (currentMillis - lastTransLogPrint >= transLogInterval &&
        radioData.control_mode == 0 &&
        cmdVel.received) {

        char buf[96];

        snprintf(
            buf,
            sizeof(buf),
            "1,%lu,TL,o=%u,tv=%d,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
            currentMillis,
            currentTransmissionOutput,
            (int)radioData.transmission_val,
            requestedTarget,
            bucket,
            currentTransmissionOutput,
            cmdVel.current_hz,
            cmdVel.linear_x
        );

        safeTextLog(buf);
        lastTransLogPrint = currentMillis;
    }

    // Optional direct CSV output; reuse cached JRK feedback.
    #if CSV_LOG_ENABLED
        char csvBuf[128];

        snprintf(
            csvBuf,
            sizeof(csvBuf),
            "log,%lu,%u,%d,%u,%d,%u",
            currentMillis,
            currentTransmissionOutput,
            (int)radioData.transmission_val,
            requestedTarget,
            bucket,
            currentTransmissionOutput
        );

        Serial.println(csvBuf);
    #endif

    lastTransmissionControlRun = currentMillis;
}


// -------------------------------------------------------------------
// Setup copied from the 0804 production firmware.  The only intended
// difference is the machine-readable firmware identity at the end.
extern "C" void setup() {
    delay(45000);  // waiting for the RPi to boot so the serial connection is made

    Serial.begin(460800);
    while (!Serial && millis() < 10000);
    Serial.println("Teensy Receiver Starting v20251225...");
    Serial.flush();

    for (int i = 0; i < 4; i++) {
        Serial.print("Debug message #");
        Serial.println(i);
        Serial.flush();
        delay(100);
    }

    Serial.println("If you see this, serial is working!");
    Serial.flush();

    Serial.println(
        "*** NEW BUILD: CSV_LOG_ENABLED=0 - No more logs! Timestamp: "
        __TIMESTAMP__
    );
    Serial.flush();

    Serial3.begin(JRK_BAUD);

    // Exit JRK safe start so motor responds to targets on power-up.
    delay(100);
    Serial3.write(0x83);
    Serial3.flush();

    // IBT-2 steering controller.
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    SPI.setSCK(13);
    SPI.setMOSI(11);
    SPI.setMISO(12);

    SPI.begin();
    delay(100);

    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        Serial.print("Radio init attempt ");
        Serial.println(i + 1);
        Serial.flush();

        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized successfully!");
            Serial.flush();
            break;
        }

        Serial.println("Radio init failed, retrying...");
        Serial.flush();
        delay(1000);
    }

    if (!initialized) {
        Serial.println("Radio hardware not responding!");
        Serial.flush();
    }

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.enableAckPayload();
    radio.setPayloadSize(14);

    radio.openWritingPipe(ADDR_TRACTOR_TO_HANDHELD);
    radio.openReadingPipe(1, ADDR_HANDHELD_TO_TRACTOR);
    radio.startListening();

    Serial.println("=== RADIO CONFIGURATION ===");
    Serial.print("Channel: ");
    Serial.println(radio.getChannel());

    Serial.print("Payload Size: ");
    Serial.println(radio.getPayloadSize());

    Serial.print("Data Rate: ");
    uint8_t dr = radio.getDataRate();
    if (dr == RF24_250KBPS) Serial.println("250KBPS");
    else if (dr == RF24_1MBPS) Serial.println("1MBPS");
    else Serial.println("2MBPS");

    Serial.print("PA Level: ");
    uint8_t pa = radio.getPALevel();
    if (pa == RF24_PA_MIN) Serial.println("MIN");
    else if (pa == RF24_PA_LOW) Serial.println("LOW");
    else if (pa == RF24_PA_HIGH) Serial.println("HIGH");
    else Serial.println("MAX");

    radio.openReadingPipe(1, ADDR_HANDHELD_TO_TRACTOR);

    Serial.print("Reading Pipe 1 Address: ");
    for (int i = 0; i < 5; i++) {
        Serial.print((char)ADDR_HANDHELD_TO_TRACTOR[i]);
    }
    Serial.println();

    Serial.print("Writing Pipe Address: ");
    for (int i = 0; i < 5; i++) {
        Serial.print((char)ADDR_TRACTOR_TO_HANDHELD[i]);
    }
    Serial.println();

    Serial.println("=========================");

    memset(&ackPayload, 0, sizeof(AckPayloadStruct));

    Serial.println("10-Bucket Control System:");
    Serial.println(
        "  transmission_val 1023 -> bucket 0 -> JRK 3138 (FULL REVERSE)"
    );
    Serial.println(
        "  transmission_val ~512 -> bucket 5 -> JRK 2836 (NEUTRAL)"
    );
    Serial.println(
        "  transmission_val 1    -> bucket 9 -> JRK 2288 (MANUAL FORWARD MAX)"
    );

    Serial.println(
        "1,0,SYS,start,fw=teensy_main_20260907,steer_hz=20"
    );
    Serial.flush();
}


// -------------------------------------------------------------------
// Main loop: identical priority/order to 0804, except it resolves to the
// 0907 controlTransmission() above.
extern "C" void loop() {
    currentMillis = millis();

    // 1. Safety
    estopCheck();

    // 2. Serial input
    parseSerialCommand();
    checkCmdVelTimeout();
    monitorSerialBuffer();

    // 3. Radio
    handleRadio();

    // 4. Control
    controlTransmission();
    controlSteering();

    debugSteerPot();
}
