# IBT-2 Gen2 — Definitive Wiring & Build Guide

**Status:** Ready to build — all circuits specified  
**Supersedes:** `05-future-ideas/IBT-2_Next_Generation.md` (planning doc, keep for history)  
**See also:** [[Teensy41_Pin_Reference]] | [[DROK-12V-to-5V-Buck-Converter]]

---

## Overview

Gen1 uses only RPWM/LPWM (pins 5/6) with R_EN and L_EN hardwired to 5V.  
Gen2 adds software control of all 6 signal pins, providing:

- **Firmware-controlled enable/disable** — no more manual fuse pull to reset overcurrent latch
- **Current monitoring** — read approximate amps and watts during operation
- **Stall detection** — cut motor automatically if pot stops moving

All new signal lines route through a perf board containing the required protection circuits.

---

## Power Architecture

```
12V Blade Fuse Block ──────────────────► IBT-2  B+ (motor power, high current)
                      └──► DROK Buck ──► IBT-2  VCC (5V logic)
                                     └──► Perf board 5V rail (pull-up resistors)

                      GND (common) ────► IBT-2  B-
                                     └──► IBT-2  GND
                                     └──► Teensy  GND
                                     └──► DROK   GND
```

**Critical:** All grounds must be tied together — previous Teensy was destroyed by
328Ω isolation between Teensy GND and JRK G2 GND.

---

## Teensy 4.1 Pin Assignments (Gen2)

| IBT-2 Pin | Teensy Pin | Direction | Circuit on perf board |
|-----------|-----------|-----------|----------------------|
| RPWM      | **5**     | OUT       | Direct wire (unchanged) |
| LPWM      | **6**     | OUT       | Direct wire (unchanged) |
| R_EN      | **7**     | OUT       | Open-drain + 10kΩ pull-up to 5V |
| L_EN      | **8**     | OUT       | Open-drain + 10kΩ pull-up to 5V |
| R_IS      | **21 (A7)** | IN (analog) | 10kΩ/10kΩ divider + 100nF filter cap |
| L_IS      | **22 (A8)** | IN (analog) | 10kΩ/10kΩ divider + 100nF filter cap |

Pins 7, 8, 21, 22 are free — no conflicts with existing firmware (NRF24 uses
9/10/11/12/13; JRK G2 uses Serial3 = TX14/RX15; steering pot uses A9/23; E-stop uses 32).

---

## Circuit 1 — EN Pin Open-Drain Pull-Up (×2)

The Teensy 4.1 runs at 3.3V logic. The IBT-2 EN pins expect 5V logic HIGH. Driving
them directly from a 3.3V Teensy output would conflict with the module's internal 5V
state. Open-drain with external pull-up solves this cleanly.

```
5V (DROK) ────── R1 (10kΩ) ────┬──── IBT-2  R_EN
                                │
                           Teensy pin 7

5V (DROK) ────── R2 (10kΩ) ────┬──── IBT-2  L_EN
                                │
                           Teensy pin 8
```

**Operation:**
- Teensy drives pin LOW (OUTPUT, LOW) → EN = 0V → IBT-2 disabled / fault latch cleared
- Teensy releases pin (INPUT mode) → pull-up takes EN to 5V → IBT-2 enabled
- Teensy never sources 5V — it only sinks to GND, which is safe at 3.3V logic

---

## Circuit 2 — IS Pin Voltage Divider + Noise Filter (×2)

The BTS7960B IS pin outputs a current equal to 1/8450 of motor current. The IBT-2
module has a 1kΩ sense resistor, so IS pin voltage = (I_motor / 8450) × 1000.

At stall (43A): V_IS = 5.09V — will destroy the Teensy 3.3V ADC input.

**Voltage divider halves the signal before it reaches the Teensy:**

```
IBT-2  R_IS ──── R3 (10kΩ) ────┬──── Teensy pin 21 (A7)
                                │
                           R4 (10kΩ) ═╗
                                │     ║ C1 (100nF) in parallel with R4
                               GND   ═╝
```

```
IBT-2  L_IS ──── R5 (10kΩ) ────┬──── Teensy pin 22 (A8)
                                │
                           R6 (10kΩ) ═╗
                                │     ║ C2 (100nF) in parallel with R6
                               GND   ═╝
```

**Why the 100nF capacitor?**  
The IBT-2 switches at PWM frequency (typically 4–8 kHz). Without filtering, the IS
voltage is a noisy square-wave-like signal riding on the average current level. A
100nF cap across the lower divider resistor creates a low-pass RC filter:

- RC = 10kΩ × 100nF = 1ms
- Cutoff frequency = 1 / (2π × RC) ≈ **159 Hz**
- PWM switching noise (4–8 kHz) is attenuated by 25–35 dB
- Current changes up to ~100 Hz still pass through cleanly

The 100nF cap is the capacitor you may recall from IBT-2 current sensing videos.
It is placed across the lower resistor (R4 / R6) — not the upper one.

**Voltage levels after divider:**

| Motor current | IS pin voltage | Teensy sees |
|---------------|---------------|-------------|
| 10A           | 1.18V         | 0.59V       |
| 20A           | 2.37V         | 1.18V       |
| 28A           | 3.31V         | 1.65V       |
| 43A (stall)   | 5.09V         | 2.55V ✓ safe |

---

## Circuit 3 — VCC Bypass Capacitor

The IBT-2 logic supply (5V VCC) needs a bulk decoupling cap to absorb spikes from
the motor switching. Place this cap as close to the IBT-2 VCC and GND pins as possible.

```
IBT-2  VCC (5V) ──┬── (to logic supply)
                  │
                 C3 (100µF / 16V electrolytic)
                  │
IBT-2  GND ───────┘
```

---

## Complete Perf Board Component List

All components are through-hole, standard sizes, available at any electronics supplier.

| Ref | Value | Type | Purpose |
|-----|-------|------|---------|
| R1  | 10kΩ 1/4W | Carbon or metal film resistor | R_EN pull-up to 5V |
| R2  | 10kΩ 1/4W | Carbon or metal film resistor | L_EN pull-up to 5V |
| R3  | 10kΩ 1/4W | Carbon or metal film resistor | R_IS divider — upper |
| R4  | 10kΩ 1/4W | Carbon or metal film resistor | R_IS divider — lower |
| R5  | 10kΩ 1/4W | Carbon or metal film resistor | L_IS divider — upper |
| R6  | 10kΩ 1/4W | Carbon or metal film resistor | L_IS divider — lower |
| C1  | 100nF (0.1µF) ceramic disc, 25V+ | Capacitor | R_IS PWM noise filter |
| C2  | 100nF (0.1µF) ceramic disc, 25V+ | Capacitor | L_IS PWM noise filter |
| C3  | 100µF / 16V electrolytic | Capacitor | VCC bulk decoupling |

**Connectors (choose based on your wiring approach):**

| Item | Purpose |
|------|---------|
| 2× 2-pin screw terminal (3.5mm pitch) | 5V input from DROK, GND |
| 1× 8-pin female header or screw terminal strip | IBT-2 signal pins (R_EN, L_EN, R_IS, L_IS, VCC, GND, and the 2 PWM passthroughs) |
| 1× 6-pin female header or jumper wires | Teensy signal pins (5, 6, 7, 8, 21, 22) |

---

## Perf Board Physical Layout

Use a standard 24×18-hole (roughly 2.4" × 1.8") or larger perf board.  
The circuits are small — 6 resistors, 3 caps, plus power buses and connectors.

```
TOP VIEW (perf board, holes are columns A–F, rows 1–10)

         IBT-2 side (left)                         Teensy side (right)
         ──────────────────                         ─────────────────

Row 1:  [GND screw term]  ─── GND bus (bottom rail across full board) ───────────────
Row 2:  [5V screw term]   ─── 5V bus  (top rail across full board)  ────────────────

     COL:  A          B              C         D              E           F

Row 3:  R_EN in ─── R1(10k) ──── pin 7 out ·  ·               ·           ·
Row 4:  L_EN in ─── R2(10k) ──── pin 8 out ·  ·               ·           ·

Row 5:  R_IS in ─── R3(10k) ─┬─ A7  out  ·  R4(10k) to GND   C1(100nF) across R4
                               │
Row 6:  L_IS in ─── R5(10k) ─┬─ A8  out  ·  R6(10k) to GND   C2(100nF) across R6

Row 7:  VCC in  ─── C3+ ────── 5V bus
                    C3- ────── GND bus

Row 8:  RPWM in ─────────────── pin 5 out  (straight passthrough)
Row 9:  LPWM in ─────────────── pin 6 out  (straight passthrough)
Row 10: GND  ──────────────────────────────────────────────── GND to Teensy

```

**Component orientation notes:**
- Resistors (R1–R6): Stand vertically or lie flat — either works, all 10kΩ so polarity doesn't matter
- Ceramic caps (C1, C2): No polarity, orient either way
- Electrolytic cap (C3): Observe polarity — longer leg = positive = to 5V rail; shorter leg = to GND

**Top and bottom bus rails:**  
Run a solder bridge or wire along the full top row for 5V and along the full bottom
row for GND. This makes connections to R1/R2 pull-ups and the cap straightforward.

---

## Firmware — Amp and Watt Calculation

The existing test firmware returns IS voltage. This conversion gives amps and watts:

```cpp
// BTS7960B: IS current = motor current / 8450
// IBT-2 module sense resistor: 1kΩ (typical — verify with multimeter on board)
// Voltage divider: ×2 correction factor

float ibt2Amps(int adcPin) {
    int raw = analogRead(adcPin);
    float vAtTeensy = (raw / 1023.0f) * 3.3f;  // ADC reading → voltage
    float vIS       = vAtTeensy * 2.0f;          // undo voltage divider
    return vIS * 8.45f;                          // V → amps (8450/1000)
}

float ibt2Watts(int adcPin, float busVolts = 12.0f) {
    return ibt2Amps(adcPin) * busVolts;
}
```

**Calibration note:** The 1kΩ sense resistor value is nominal. If readings seem off,
measure the actual sense resistor on your IBT-2 board (it is labeled near the IS pins)
and adjust the multiplier: `amps = vIS × (8450.0 / R_sense_ohms)`.

**Stall threshold (from Gen2 test firmware):** ADC value 512 ≈ 1.65V at Teensy pin ≈
3.3V actual IS ≈ 27.9A — a reasonable stall warning threshold. Tune from field data.

---

## Wiring Checklist Before Power-On

- [ ] All grounds tied: Teensy GND, IBT-2 GND, IBT-2 B-, DROK GND, blade fuse block GND
- [ ] 12V from blade fuse block to IBT-2 B+ only (high-current, not on perf board)
- [ ] DROK 5V output to perf board 5V rail and IBT-2 VCC
- [ ] RPWM/LPWM (pins 5/6) still connected as before
- [ ] R_EN → perf board → Teensy pin 7
- [ ] L_EN → perf board → Teensy pin 8
- [ ] R_IS → perf board upper divider → filtered output → Teensy pin 21 (A7)
- [ ] L_IS → perf board upper divider → filtered output → Teensy pin 22 (A8)
- [ ] C3 electrolytic polarity correct (+ to 5V)
- [ ] Sense resistor value verified on IBT-2 board

---

## Firmware Integration Plan

Before integrating into `teensy_main_20260518.cpp`:
1. Test with standalone Gen2 test firmware in `tractor_rpi/testing/teensy41_ibt2_gen2/`
2. Confirm IS readings look reasonable at idle vs. under load vs. stall
3. Then add `ibt2Enable()`, `ibt2Disable()`, `ibt2ClearFault()` functions to main firmware
4. Add `ibt2Amps()` and stall detection to `controlSteering()`
5. Add IS readings to Serial status broadcast so RPi can log current draw
