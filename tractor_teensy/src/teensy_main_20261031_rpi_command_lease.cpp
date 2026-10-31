/*********************************************************************
  teensy_main_20261031_rpi_command_lease.cpp

  Additive, opt-in RPi-supervised firmware candidate. This file reuses the
  frozen 2026-09-14 steering, transmission, JRK, NRF, e-stop, and telemetry
  implementation. It replaces only the active serial parser and top-level
  authority decision.

  The Teensy does not evaluate phone or network state. It accepts one generic
  RPi command frame and enforces a fixed local command lease no longer than
  the existing 500 ms cmd_vel watchdog:

    RPICMD,<uint32 session>,<uint32 sequence>,<mode>,<drive>,<steering>*<CRC16-CCITT>

      mode 0: Pause; drive=0 and steering=0
      mode 1: Manual; drive=-100..100 percent, steering=-1..1
      mode 2: Auto; drive=0..1.8 m/s, steering=-1..1

  Invalid, corrupt, duplicate, or reordered frames do not refresh the lease.
  A new RPi session is accepted only as a zero Pause claim while the physical
  handheld is in Pause, so an RPi restart cannot silently re-arm motion.
  Lease expiry latches Pause until a fresh explicit RPi Pause frame arrives;
  delayed motion frames therefore cannot automatically resume the tractor.
  Lease expiry, physical Pause, physical e-stop, and existing local actuator
  faults retain immediate Teensy authority.

  This candidate is not selected by platformio.ini and is not field approved.
*********************************************************************/

#define TRACTOR_FIRMWARE_ID "teensy_main_20261031_rpi_command_lease_v1"
#define TRACTOR_TOP_LEVEL_LOOP loop_20260914_frozen_reference
#include "teensy_main_20260914.cpp"
#undef TRACTOR_TOP_LEVEL_LOOP


enum RpiCommandMode : byte {
    RPI_MODE_PAUSE = 0,
    RPI_MODE_MANUAL_PERCENT = 1,
    RPI_MODE_AUTO_MPS = 2,
};

enum RpiRejectReason : byte {
    RPI_REJECT_NONE = 0,
    RPI_REJECT_CHECKSUM = 1,
    RPI_REJECT_FORMAT = 2,
    RPI_REJECT_RANGE = 3,
    RPI_REJECT_SEQUENCE = 4,
    RPI_REJECT_UNSUPPORTED = 5,
    RPI_REJECT_SESSION = 6,
};

static const unsigned long RPI_COMMAND_LEASE_MS = CMD_VEL_TIMEOUT;
static_assert(RPI_COMMAND_LEASE_MS <= 500UL, "RPi command lease may not exceed 500 ms");

static bool rpiCommandSelected = false;
static bool rpiSessionSeen = false;
static uint32_t rpiSession = 0;
static bool rpiSequenceSeen = false;
static uint32_t rpiSequence = 0;
static byte rpiCommandMode = RPI_MODE_PAUSE;
static float rpiDriveCommand = 0.0f;
static float rpiSteeringCommand = 0.0f;
static unsigned long rpiCommandTimestamp = 0;
static bool rpiLeaseExpiredLatched = false;
static unsigned long rpiAcceptedCount = 0;
static unsigned long rpiRejectedCount = 0;
static byte rpiLastRejectReason = RPI_REJECT_NONE;


static uint16_t rpiCrc16Ccitt(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (byte bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000)
                ? (uint16_t)((crc << 1) ^ 0x1021)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}


static bool parseFourHexDigits(const char* text, uint16_t* value) {
    if (!text || !value || strlen(text) != 4) return false;
    uint16_t parsed = 0;
    for (byte i = 0; i < 4; i++) {
        const char c = text[i];
        byte nibble;
        if (c >= '0' && c <= '9') nibble = (byte)(c - '0');
        else if (c >= 'A' && c <= 'F') nibble = (byte)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') nibble = (byte)(c - 'a' + 10);
        else return false;
        parsed = (uint16_t)((parsed << 4) | nibble);
    }
    *value = parsed;
    return true;
}


static bool rpiSequenceIsNewer(uint32_t candidate, uint32_t previous) {
    const uint32_t difference = candidate - previous;
    return difference != 0 && difference < 0x80000000UL;
}


static bool parseUint32Token(const char* text, uint32_t* value) {
    if (!text || !*text || !value) return false;
    uint32_t parsed = 0;
    for (const char* cursor = text; *cursor; cursor++) {
        if (*cursor < '0' || *cursor > '9') return false;
        const uint32_t digit = (uint32_t)(*cursor - '0');
        if (parsed > (0xFFFFFFFFUL - digit) / 10UL) return false;
        parsed = parsed * 10UL + digit;
    }
    *value = parsed;
    return true;
}


static bool parseFiniteFloatToken(const char* text, float* value) {
    if (!text || !*text || !value) return false;
    char* end = nullptr;
    const float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}


static int normalizedSteerToRadioForRpi(float normalized) {
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


static void rejectRpiCommand(byte reason) {
    rpiRejectedCount++;
    rpiLastRejectReason = reason;
}


static bool acceptRpiCommandLine(char* line) {
    if (strncmp(line, "RPICMD,", 7) != 0) return false;

    char* separator = strrchr(line, '*');
    if (!separator || separator == line || strchr(separator + 1, '*')) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }

    uint16_t suppliedCrc = 0;
    if (!parseFourHexDigits(separator + 1, &suppliedCrc)) {
        rejectRpiCommand(RPI_REJECT_CHECKSUM);
        return true;
    }
    const size_t bodyLength = (size_t)(separator - line);
    if (rpiCrc16Ccitt(line, bodyLength) != suppliedCrc) {
        rejectRpiCommand(RPI_REJECT_CHECKSUM);
        return true;
    }
    *separator = '\0';

    char* fields[5];
    char* cursor = line + 7;
    for (byte i = 0; i < 4; i++) {
        fields[i] = cursor;
        char* comma = strchr(cursor, ',');
        if (!comma) {
            rejectRpiCommand(RPI_REJECT_FORMAT);
            return true;
        }
        *comma = '\0';
        cursor = comma + 1;
    }
    fields[4] = cursor;
    if (strchr(fields[4], ',')) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }

    uint32_t parsedSession = 0;
    uint32_t parsedSequence = 0;
    float parsedDrive = 0.0f;
    float parsedSteering = 0.0f;
    if (!parseUint32Token(fields[0], &parsedSession) ||
        !parseUint32Token(fields[1], &parsedSequence) ||
        strlen(fields[2]) != 1 || fields[2][0] < '0' || fields[2][0] > '2' ||
        !parseFiniteFloatToken(fields[3], &parsedDrive) ||
        !parseFiniteFloatToken(fields[4], &parsedSteering)) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }
    const byte parsedMode = (byte)(fields[2][0] - '0');
    if (parsedSteering < -1.0f || parsedSteering > 1.0f) {
        rejectRpiCommand(RPI_REJECT_RANGE);
        return true;
    }
    if ((parsedMode == RPI_MODE_PAUSE &&
         (parsedDrive != 0.0f || parsedSteering != 0.0f)) ||
        (parsedMode == RPI_MODE_MANUAL_PERCENT &&
         (parsedDrive < -100.0f || parsedDrive > 100.0f)) ||
        (parsedMode == RPI_MODE_AUTO_MPS &&
         (parsedDrive < 0.0f || parsedDrive > 1.8f))) {
        rejectRpiCommand(RPI_REJECT_RANGE);
        return true;
    }

    const bool sessionChanged = !rpiSessionSeen || parsedSession != rpiSession;
    const bool physicalPauseForSessionClaim =
        radioStats.signalGood && radioData.control_mode == 2;
    if (sessionChanged &&
        (!physicalPauseForSessionClaim || parsedMode != RPI_MODE_PAUSE)) {
        rejectRpiCommand(RPI_REJECT_SESSION);
        return true;
    }
    if (!sessionChanged && rpiSequenceSeen &&
        !rpiSequenceIsNewer(parsedSequence, rpiSequence)) {
        rejectRpiCommand(RPI_REJECT_SEQUENCE);
        return true;
    }

    rpiSessionSeen = true;
    rpiSession = parsedSession;
    rpiSequenceSeen = true;
    rpiSequence = parsedSequence;
    rpiCommandMode = parsedMode;
    rpiDriveCommand = parsedDrive;
    rpiSteeringCommand = parsedSteering;
    rpiCommandTimestamp = millis();
    rpiCommandSelected = true;
    if (rpiCommandMode == RPI_MODE_PAUSE) rpiLeaseExpiredLatched = false;
    rpiAcceptedCount++;
    rpiLastRejectReason = RPI_REJECT_NONE;

    if (rpiCommandMode == RPI_MODE_PAUSE) {
        cmdVel.linear_x = 0.0f;
        cmdVel.angular_z = 0.0f;
        cmdVel.received = false;
    } else if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT) {
        cmdVel.linear_x = 0.0f;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    } else {
        cmdVel.linear_x = rpiDriveCommand;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    }
    cmdVel.message_count++;
    last_cmd_vel_time = rpiCommandTimestamp;
    return true;
}


static void parseRpiCommandLeaseSerial() {
    if (currentMillis - lastSerialProcessTime < serialProcessInterval) return;

    const unsigned long startTime = millis();
    static char buffer[96];
    static uint8_t index = 0;
    static unsigned long lastRateCalc = 0;
    static unsigned long messagesSinceRate = 0;
    int processed = 0;

    while (Serial.available() > 0 && processed < 128 &&
           (millis() - startTime) < maxSerialProcessTime) {
        const char c = Serial.read();
        processed++;

        if (c == '\n') {
            buffer[index] = '\0';
            if (index > 0) {
                if (acceptRpiCommandLine(buffer)) {
                    if (rpiLastRejectReason == RPI_REJECT_NONE) messagesSinceRate++;
                } else if (index >= 4 && memcmp(buffer, "GPS,", 4) == 0) {
                    int status = -1;
                    char trailing = '\0';
                    if (sscanf(buffer + 4, "%d%c", &status, &trailing) == 1 &&
                        status >= 0 && status <= 3) {
                        updateGpsStatus((byte)status);
                    } else {
                        rejectRpiCommand(RPI_REJECT_RANGE);
                    }
                } else {
                    // Legacy CMD and WIFI motion frames are intentionally not
                    // authority inputs in this opt-in firmware.
                    rejectRpiCommand(RPI_REJECT_UNSUPPORTED);
                }
            }
            index = 0;
        } else if (c != '\r' && index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        } else if (c != '\r') {
            index = 0;
            serialStats.overrunCount++;
            rejectRpiCommand(RPI_REJECT_FORMAT);
        }
    }

    if (currentMillis - lastRateCalc >= 1000) {
        const float elapsed = (currentMillis - lastRateCalc) / 1000.0f;
        cmdVel.current_hz = messagesSinceRate / elapsed;
        messagesSinceRate = 0;
        lastRateCalc = currentMillis;
    }
    lastSerialProcessTime = currentMillis;
}


static void publishRpiCommandLeaseStatus(bool arrivalFresh, bool leaseUsable) {
    static unsigned long lastStatusPrint = 0;
    if (currentMillis - lastStatusPrint < 200) return;
    const unsigned long age = rpiCommandSelected
        ? currentMillis - rpiCommandTimestamp
        : 999999UL;
    char buffer[160];
    snprintf(
        buffer,
        sizeof(buffer),
        "1,%lu,RPI_CTL,m=%u,f=%d,u=%d,l=%d,a=%lu,ss=%lu,s=%lu,ok=%lu,rj=%lu,rr=%u",
        currentMillis,
        (unsigned int)rpiCommandMode,
        arrivalFresh ? 1 : 0,
        leaseUsable ? 1 : 0,
        rpiLeaseExpiredLatched ? 1 : 0,
        age,
        (unsigned long)rpiSession,
        (unsigned long)rpiSequence,
        rpiAcceptedCount,
        rpiRejectedCount,
        (unsigned int)rpiLastRejectReason
    );
    Serial.println(buffer);
    lastStatusPrint = currentMillis;
}


extern "C" void loop() {
    currentMillis = millis();

    // Existing local e-stop relay authority remains first.
    estopCheck();

    // Only the generic RPi frame and non-motion GPS status are active here.
    parseRpiCommandLeaseSerial();
    currentMillis = millis();
    monitorSerialBuffer();
    publishPeriodicSystemIdentity();

    // Preserve NRF telemetry and physical controls. Once an RPi command has
    // been selected, a received physical Pause remains an immediate override.
    handleRadio();
    const bool savedSignalGood = radioStats.signalGood;
    const byte savedMode = radioData.control_mode;
    const int16_t savedSteering = radioData.steering_val;
    const int16_t savedTransmission = radioData.transmission_val;
    const bool physicalPause = savedSignalGood && savedMode == 2;

    const bool arrivalFresh = rpiCommandSelected &&
        currentMillis - rpiCommandTimestamp <= RPI_COMMAND_LEASE_MS;
    if (rpiCommandSelected && !arrivalFresh) rpiLeaseExpiredLatched = true;
    const bool leaseUsable = arrivalFresh && !rpiLeaseExpiredLatched;

    if (rpiCommandSelected) {
        // RPi control does not require an NRF link, matching the proven direct
        // control behavior. Physical Pause still wins whenever received.
        radioStats.signalGood = true;

        if (physicalPause || !leaseUsable || rpiCommandMode == RPI_MODE_PAUSE) {
            radioData.control_mode = 2;
            wifiDriveCommand = true;
            wifiDrivePercent = 0.0f;
            cmdVel.received = false;
        } else if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT) {
            radioData.control_mode = 0;
            wifiDriveCommand = true;
            wifiDrivePercent = rpiDriveCommand;
            wifiManualDrivePercent = rpiDriveCommand;
            lastMotionCommandWasWifi = true;
            cmdVel.linear_x = 0.0f;
            cmdVel.angular_z = rpiSteeringCommand;
            cmdVel.timestamp = rpiCommandTimestamp;
            cmdVel.received = true;
        } else {
            radioData.control_mode = 0;
            wifiDriveCommand = false;
            wifiDrivePercent = 0.0f;
            lastMotionCommandWasWifi = false;
            cmdVel.linear_x = rpiDriveCommand;
            cmdVel.angular_z = rpiSteeringCommand;
            cmdVel.timestamp = rpiCommandTimestamp;
            cmdVel.received = true;
        }

        // Preserve the proven Pause -> neutral Manual steering-fault recovery.
        if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT && leaseUsable &&
            steeringFaultLatched && steeringRecoveryPauseSeen &&
            fabsf(rpiDriveCommand) < 0.5f) {
            radioData.control_mode = 1;
            radioData.transmission_val = 600;
            radioData.steering_val = normalizedSteerToRadioForRpi(
                rpiSteeringCommand
            );
        }
    }

    controlTransmission();
    controlSteering();
    publishRpiCommandLeaseStatus(arrivalFresh, leaseUsable);

    // Keep separately published NRF diagnostics truthful.
    radioStats.signalGood = savedSignalGood;
    radioData.control_mode = savedMode;
    radioData.steering_val = savedSteering;
    radioData.transmission_val = savedTransmission;

    debugSteerPot();
}
