# NRF24L01 Radio Configuration
**Source files:** `Rccntrl_rf24_20260517.cpp` (handheld) · `teensy_main_20260517.cpp` (tractor)
**Last updated:** 2026-05-17

> **Hard-won lesson:** Mismatched pipe addresses between the two radios caused a full day of debugging. The addresses must be set as mirror images — what the handheld writes to, the tractor reads from, and vice versa. See the address table below.

---

## RF Settings (must match on both sides)

| Setting | Value |
|---------|-------|
| Library | RF24 |
| Channel | **76** |
| Data Rate | **RF24_250KBPS** |
| PA Level | RF24_PA_HIGH |
| Payload Size | **14 bytes** |
| ACK Payload | Enabled |
| Transmit Rate | 10 Hz (every 100 ms) |

---

## Pipe Addresses — The Critical Part

These are the "keys" that must mirror each other. A mismatch here will silently fail.

| Address Name | String | Used by Handheld | Used by Tractor |
|---|---|---|---|
| `ADDR_HANDHELD_TO_TRACTOR` | `"1Node"` | **Writing pipe** (TX) | Reading pipe 1 (RX) |
| `ADDR_TRACTOR_TO_HANDHELD` | `"2Node"` | Reading pipe 1 (RX) | **Writing pipe** (TX) |

**Rule:** Each side's writing address is the other side's reading address.

---

## Hardware Pins

### Handheld RC Unit (Transmitter) Teensy 3.2

| Signal | Pin |
|--------|-----|
| CE | 7 |
| CSN | 8 |
| Mode | stopListening() — PTX |

### Tractor Teensy 4.1 (Receiver)
| Signal | Pin |
|--------|-----|
| CE | 9 |
| CSN | 10 |
| SPI SCK | 13 |
| SPI MOSI | 11 |
| SPI MISO | 12 |
| Mode | startListening() — PRX |

---

## Data Structures (must be identical on both sides)

### RadioControlStruct — Handheld → Tractor (14 bytes, packed)

| Field | Type | Bytes | Source Pin | Notes |
|-------|------|-------|------------|-------|
| `steering_val` | int16_t | 2 | Pin 16 (RC) / A9 (Teensy) | 0–1024 range |
| `throttle_val` | int16_t | 2 | Pin 14 | 0–1024 range |
| `transmission_val` | int16_t | 2 | Pin 15 | 0–1024 → 10-bucket system |
| `voltage_mv` | uint16_t | 2 | Pin 18 | Millivolts (e.g. 12600 = 12.6V) |
| `pot4_val` | int16_t | 2 | Pin 17 | Spare pot |
| `estop` | byte | 1 | Pin 10 | 1 = E-stop active (active LOW input) |
| `control_mode` | byte | 1 | Pins 3 & 4 | 0=Pause, 1=Manual, 2=Auto |
| `button02` | byte | 1 | Pin 9 | Active LOW input |
| `button03` | byte | 1 | Pin 6 | Active LOW input |
| **Total** | | **14** | | |

### AckPayloadStruct — Tractor → Handheld (14 bytes, packed)

| Field | Type | Bytes | Notes |
|-------|------|-------|-------|
| `gps_status` | byte | 1 | 0=unset, 1=no NMEA, 2=GPS no RTK, 3=RTK Fix |
| `button02_status` | byte | 1 | Echo of received button02 |
| `button03_status` | byte | 1 | Echo of received button03 |
| `padding[11]` | byte | 11 | Zeroed — pads struct to 14 bytes |
| **Total** | | **14** | Must match payload size setting |

> **Why padding?** The NRF24 payload size is fixed at 14 bytes. Both structs must be exactly 14 bytes or the receiver reads garbage.

---

## Control Modes

| Mode Value | Switch Position | Behavior |
|---|---|---|
| 0 | Left | **Pause** — transmission neutral, steering holds position |
| 1 | Right | **Manual** — RC pots directly control steering and transmission |
| 2 | Center | **Auto** — cmd_vel from RPi5 navigation stack |

If signal is lost (no ACK for >2 seconds), tractor forces mode=9 (safety fallback → neutral/stop).

---

## Signal Quality (Handheld LED #1)

| ACK Rate | LED Color | Status |
|---|---|---|
| > 5 Hz | Green | Good |
| 2–5 Hz | Orange | Moderate |
| < 2 Hz | Red | Poor |

Signal timeout: **2000 ms** — if no ACK received within 2s, `signalGood = false`.

---

## Voltage Monitoring

- Voltage divider ratio: **5.0** (e.g. 40kΩ / 10kΩ)
- ADC reference: **3.3V**
- Transmitted as millivolts in `voltage_mv` (uint16_t)
- Example: 12.6V → 12600 mv
- Handheld LED #4 (repurposed): now shows **GPS status** from ACK payload, not voltage

---

## Startup Behavior

- Handheld retries radio init up to **5 times** before halting with flashing red LED
- Tractor: **45-second boot delay** at startup (waits for RPi5 serial connection)
- Tractor retries radio init up to **5 times**, then continues anyway with log warnings
- Both sides print full radio config to serial on startup for verification

---

## Troubleshooting Checklist

- [ ] Channel matches on both sides (must be **76**)
- [ ] Addresses are mirrored correctly (not the same on both sides)
- [ ] Payload size matches struct size (**14 bytes** both sides)
- [ ] Data rate matches (**RF24_250KBPS** both sides)
- [ ] `enableAckPayload()` called on both sides
- [ ] Handheld: `stopListening()` (PTX mode)
- [ ] Tractor: `startListening()` (PRX mode)
- [ ] SPI initialized before `radio.begin()`
- [ ] All grounds tied together 
