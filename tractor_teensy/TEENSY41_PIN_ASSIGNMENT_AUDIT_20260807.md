# Tractor01 Teensy 4.1 pin-assignment audit

Date reviewed: 2026-08-07

## Authority and scope

PlatformIO selects only `tractor_teensy/src/teensy_main_20260804.cpp`. This audit lists pins referenced by that source plus Teensy 4.1 pins implied by `SPI` and `Serial3`. Archived firmware and test sketches are not active tractor01 assignments.

## Active pin assignments

| Pin | Firmware symbol/API | Direction | Device and behavior |
|---:|---|---|---|
| 2 | `DATA_PIN` | Declared only | NeoPixel described as unused; no active configuration or output call. |
| 5 | `RPWM_Output` | PWM OUT | IBT-2 steering RPWM; decreasing pot counts/right. |
| 6 | `LPWM_Output` | PWM OUT | IBT-2 steering LPWM; increasing pot counts/left. |
| 9 | `RF24 radio(9, 10)` CE | OUT | NRF24 chip enable. |
| 10 | `RF24 radio(9, 10)` CSN | OUT | NRF24 SPI chip select. |
| 11 | `SPI.setMOSI(11)` | OUT | NRF24 SPI0 MOSI. |
| 12 | `SPI.setMISO(12)` | IN | NRF24 SPI0 MISO. |
| 13 | `SPI.setSCK(13)` | OUT | NRF24 SPI0 clock; also onboard LED pin. |
| 14 | `Serial3` TX3 | UART OUT | Commands to JRK RX at 9600 baud. |
| 15 | `Serial3` RX3 | UART IN | Responses from JRK TX at 9600 baud. |
| 23 / A9 | `STEER_POT_PIN` | Analog IN | Steering-pot wiper; right=197, center=447, left=815. |
| 30 | `ESTOP_RELAY_PIN` | OUT | E-stop relay; HIGH normal/startup, LOW when E-stop asserted. |

USB serial runs at 460800 baud through the USB connector, not an edge GPIO pin.

## Non-GPIO connections requiring physical verification

Firmware cannot show actual power/ground wiring. Verify Teensy power/ground, JRK logic ground, JRK feedback reference/wiper/ground, NRF24 supply/ground, steering-pot 3.3 V/ground, IBT-2 logic ground, and relay-board supply/ground/polarity.

Teensy 4.1 GPIO and analog inputs are 3.3 V devices. Do not apply 5 V to A9 or another signal pin.

## Planned or historical pins not active now

| Pins | Documented purpose | Active status |
|---|---|---|
| 7, 8 | IBT-2 Gen2 R_EN/L_EN | Not referenced or configured. |
| 21/A7, 22/A8 | IBT-2 Gen2 R_IS/L_IS | Not referenced or configured. |
| 32 | E-stop in old documentation | Incorrect for active firmware; use pin 30. |
| Ethernet bottom pads | Native Ethernet | Not configured here and do not consume edge pins 0-41. |

## Documents found in the repository

### Current pin reference

`obsidian_vault/03-design/03-3-tractor/03-3-2-teensy-4-1/Teensy41_Pin_Reference.md`

Best existing pin-focused document, but it is not fully current. It lists planned IBT-2 Gen2 pins as active and incorrectly lists E-stop pin 32; active firmware uses pin 30. Use this audit when rewiring.

`obsidian_vault/03-design/03-3-tractor/03-3-2-teensy-4-1/teensy_41_pinout.pdf`

PJRC board pinout for physical locations and alternate functions; it does not describe tractor-specific wiring.

### Related system documentation

`obsidian_vault/navigation_flow_20260714.md`

Correctly describes steering pins 5/6, A9, USB serial, and the JRK signal path, but is not a complete connector checklist and predates the latest filename.

### Obsolete or stale documentation

`tractor_teensy/tractor_teensy_main_documentation.md`

January 2025 Teensy 3.5-era document. It contains E-stop pin 32, old modes, baud rates, radio details, control rates, and transmission targets. Do not use it to wire tractor01.

`firmware_versions_20260702.md`

Deployment history rather than a pin map. Its production table does not record the confirmed 2026-08-04 firmware flash.

Archived source under `tractor_teensy/src/archive/` and sketches under `tractor_rpi/testing/` contain historical/test pin assignments and are not current wiring authorities.

## Latest firmware review

1. Active pins are internally consistent: steering 5/6/A9, radio 9-13, JRK Serial3 14/15, relay 30.
2. No active support exists for IBT-2 Gen2 enable/current-sense wiring.
3. `DATA_PIN 2` and `NUM_LEDS 1` are declarations only; no NeoPixel is driven.
4. E-stop is active LOW and initializes HIGH. Verify actual relay polarity and perform a physical test.
5. `Serial3.write(0x83)` is incorrectly commented as exit-safe-start. In JRK G2 compact protocol it requests a target variable byte. Correct/remove it in a later controlled firmware change.
6. JRK analog feedback terminates at the JRK and therefore does not appear as a Teensy GPIO assignment.

## Remount checklist

- Label both wire ends with pin number and function.
- Add strain relief so vibration and harness weight do not load joints.
- Keep JRK feedback/reference/ground together and away from motor-current wiring where practical.
- Verify pins 14 and 15 are crossed correctly at the JRK.
- Verify pin 30, not 32, reaches the relay input.
- Continuity-test grounds with power removed.
- Check adjacent pins for shorts after soldering.
- Photograph and document standardized wire colors and connector positions.
