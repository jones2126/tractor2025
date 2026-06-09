/*********************************************************************
  jrk_range_explorer_20260609.cpp
  --------------------------------------------------------------
  Interactive keypress-controlled actuator range exploration.
  YOU control each step -- nothing moves without your input.

  COMMANDS (type in serial monitor, press Enter):
    f  = step FORWARD  50 counts (decreases target value)
    r  = step REVERSE  50 counts (increases target value)
    p  = print current target and feedback
    h  = return to HOME position (2888)
    q  = quit -- holds current position, stops accepting commands

  STARTING POSITION: 2888 (confirmed resting position 2026-06-09)

  SAFETY:
    - Start with actuator disconnected if unsure
    - Each step is only 50 counts -- small incremental moves
    - Watch linkage carefully before pressing f or r
    - Press h at any time to return to home
    - Power off immediately if binding or resistance felt

  WIRING:
    Serial3 TX (pin 8) --> JRK RX
    Serial3 RX (pin 7) --> JRK TX
    Common GND between Teensy and JRK
*********************************************************************/

#include <Arduino.h>

#define JRK_BAUD     9600
#define JRK_TIMEOUT  25
#define STEP_SIZE    50     // counts per keypress

// Safety limits -- do not command outside these bounds
// Set conservatively; expand only after confirming physical limits
#define TARGET_MIN   2200   // forward limit (lower = more forward)
#define TARGET_MAX   3600   // reverse limit (higher = more reverse)

const uint16_t HOME_POSITION = 2888;  // confirmed resting position

uint16_t currentTarget = HOME_POSITION;
bool motorRunning = false;

// -------------------------------------------------------------------
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    Serial3.flush();
}

void exitSafeStart() {
    Serial3.write(0x83);
    Serial3.flush();
}

uint16_t readJrkFeedback() {
    while (Serial3.available()) Serial3.read();
    Serial3.write(0xE5);
    Serial3.write(0x04);
    Serial3.write(0x02);
    Serial3.flush();
    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > JRK_TIMEOUT) return 0xFFFF;
    }
    uint8_t lo = Serial3.read();
    uint8_t hi = Serial3.read();
    return (uint16_t)((hi << 8) | lo);
}

void printStatus(const char* label) {
    uint16_t feedback = readJrkFeedback();
    Serial.print(label);
    Serial.print("  target="); Serial.print(currentTarget);
    Serial.print("  feedback=");
    if (feedback == 0xFFFF) {
        Serial.print("TIMEOUT");
    } else {
        Serial.print(feedback);
        int error = (int)feedback - (int)currentTarget;
        Serial.print("  error=");
        Serial.print(error > 0 ? "+" : "");
        Serial.print(error);
    }
    Serial.println();
}

void printHelp() {
    Serial.println();
    Serial.println("Commands (type letter + Enter):");
    Serial.println("  f = step FORWARD  50 counts (target decreases)");
    Serial.println("  r = step REVERSE  50 counts (target increases)");
    Serial.println("  p = print current target and feedback");
    Serial.println("  h = return to HOME (2888)");
    Serial.println("  q = quit (hold position, stop accepting input)");
    Serial.print  ("  Safety limits: MIN="); Serial.print(TARGET_MIN);
    Serial.print  ("  MAX="); Serial.println(TARGET_MAX);
    Serial.println();
}

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("==================================================");
    Serial.println("  JRK Range Explorer v20260609");
    Serial.println("  Keypress controlled -- YOU approve each move");
    Serial.println("==================================================");
    Serial.print  ("  Home position : "); Serial.println(HOME_POSITION);
    Serial.print  ("  Step size     : "); Serial.println(STEP_SIZE);
    Serial.print  ("  Forward limit : "); Serial.println(TARGET_MIN);
    Serial.print  ("  Reverse limit : "); Serial.println(TARGET_MAX);
    Serial.println("==================================================");

    Serial3.begin(JRK_BAUD);
    delay(100);
    exitSafeStart();
    delay(100);

    // Command home position on startup
    currentTarget = HOME_POSITION;
    setJrkTarget(currentTarget);

    Serial.println();
    Serial.println("Commanding HOME position...");
    delay(2000);  // give actuator time to settle
    printStatus("  Startup status:");
    printHelp();
    Serial.println("Waiting for command...");
}

// -------------------------------------------------------------------
void loop() {
    // Read and echo feedback every 3 seconds passively
    static unsigned long lastPassivePrint = 0;
    if (millis() - lastPassivePrint >= 3000) {
        printStatus("  [auto]");
        lastPassivePrint = millis();
    }

    // Check for keypress
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        // Flush rest of input (e.g. newline)
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case 'f':
            case 'F': {
                uint16_t newTarget = currentTarget - STEP_SIZE;
                if (newTarget < TARGET_MIN) {
                    Serial.print("  LIMIT: already at forward limit (");
                    Serial.print(TARGET_MIN);
                    Serial.println("). Command ignored.");
                } else {
                    currentTarget = newTarget;
                    setJrkTarget(currentTarget);
                    Serial.print("  >> FORWARD step: ");
                    delay(1500);  // wait for actuator to settle
                    printStatus("");
                }
                break;
            }

            case 'r':
            case 'R': {
                uint16_t newTarget = currentTarget + STEP_SIZE;
                if (newTarget > TARGET_MAX) {
                    Serial.print("  LIMIT: already at reverse limit (");
                    Serial.print(TARGET_MAX);
                    Serial.println("). Command ignored.");
                } else {
                    currentTarget = newTarget;
                    setJrkTarget(currentTarget);
                    Serial.print("  >> REVERSE step: ");
                    delay(1500);  // wait for actuator to settle
                    printStatus("");
                }
                break;
            }

            case 'p':
            case 'P':
                printStatus("  [manual read]");
                break;

            case 'h':
            case 'H':
                currentTarget = HOME_POSITION;
                setJrkTarget(currentTarget);
                Serial.print("  >> Returning to HOME: ");
                delay(2000);
                printStatus("");
                break;

            case 'q':
            case 'Q':
                Serial.println("  >> QUIT -- holding current position.");
                Serial.print  ("  Final target  : "); Serial.println(currentTarget);
                printStatus("  Final status  :");
                Serial.println("  Power off or reflash when done.");
                while (true) delay(1000);  // hold forever
                break;

            case '\n':
            case '\r':
                // ignore bare newlines
                break;

            default:
                Serial.print("  Unknown command: '");
                Serial.print(cmd);
                Serial.println("'  -- type f, r, p, h, or q");
                break;
        }

        if (cmd != 'q' && cmd != 'Q' && cmd != '\n' && cmd != '\r') {
            Serial.println("Waiting for command...");
        }
    }
}
