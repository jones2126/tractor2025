# IBT-2 — Next Generation Wiring

**Status:** Planning only — not yet installed on tractor (as of 2026-05-19)
**Current version:** pins 5, 6 only (RPWM, LPWM) — R_EN/L_EN hardwired to 5V
**Gen2 goal:** Full pin control for fault detection, stall detection, and IBT-2 reset

See also: [[Teensy41_Pin_Reference]] | [[Teensy41_Ethernet_Datalogger]]

---

## Why Gen2 Is Needed

During field testing (2026-05-18), the BTS7960B overcurrent protection latched off twice:
- At hard left stop: full PWM=255 caused instant overcurrent → 0V output, motor silent
- After 2 min stall: thermal shutdown → 0V output

**Root cause:** R_EN and L_EN were hardwired to 5V. No way for firmware to reset the chip after a fault. The only reset was to pull the motor fuse for 5 seconds.

**Gen2 fixes:**
1. Connect R_EN and L_EN to Teensy output pins → firmware can reset the IBT-2 without human intervention
2. Connect R_IS and L_IS to Teensy analog pins → firmware can detect stall current before thermal shutdown
3. Add stall detection: if pot doesn't move for 2–3 seconds while PWM is applied → cut motor, log fault, recover

---

## IBT-2 (BTS7960B) Pin Reference

| IBT-2 Pin | Function | Signal type |
|-----------|----------|-------------|
| RPWM | Right channel PWM | Digital input from Teensy |
| LPWM | Left channel PWM | Digital input from Teensy |
| R_EN | Right channel enable | Digital input (HIGH = enabled) |
| L_EN | Left channel enable | Digital input (HIGH = enabled) |
| R_IS | Right channel current sense | Analog output to Teensy |
| L_IS | Left channel current sense | Analog output to Teensy |
| VCC | Logic supply | 5V |
| GND | Ground | 0V |
| B+ | Motor power | 12–14V |
| B- | Motor power ground | 0V |
| M+ | Motor output | To steering motor |
| M- | Motor output | To steering motor |

---

## Gen2 Teensy 4.1 Pin Assignments

| IBT-2 Pin | Teensy Pin | Direction | Notes |
|-----------|-----------|-----------|-------|
| RPWM | **5** | OUT | Unchanged from Gen1 |
| LPWM | **6** | OUT | Unchanged from Gen1 |
| R_EN | **7** | OUT | Open-drain — see wiring below |
| L_EN | **8** | OUT | Open-drain — see wiring below |
| R_IS | **21 (A7)** | IN (analog) | Voltage divider — see wiring below |
| L_IS | **22 (A8)** | IN (analog) | Voltage divider — see wiring below |

---

## 3.3V Safety — Critical Wiring Details

The Teensy 4.1 runs at **3.3V logic**. The IBT-2 module's VCC is 5V.

### R_EN and L_EN — Open-Drain with Pull-Up

The BTS7960B logic HIGH threshold is ~1.75V, but the IBT-2 module may have
internal pull-ups to 5V. Driving EN pins directly from 3.3V Teensy output
could conflict with the module's internal 5V pull.

**Solution: open-drain with external pull-up**

```
IBT-2 R_EN ──┬── 10kΩ ── 5V
             │
         Teensy pin 7
```

- **Teensy drives LOW (0V):** EN = 0V → IBT-2 disabled / fault cleared
- **Teensy releases (INPUT or HIGH):** EN pulled to 5V by resistor → IBT-2 enabled
- Teensy never needs to source 5V — it only sinks to GND

Repeat same circuit for L_EN on pin 8.

### R_IS and L_IS — Voltage Divider

The IS pins output a voltage proportional to motor current. At stall this can
reach 5V — dangerous to Teensy 3.3V ADC.

**Voltage at IS pin:**
- IS current ratio: 1/8450 of motor current
- IBT-2 module sense resistor: typically 1kΩ
- At 28A: V = (28/8450) × 1000 = **3.31V** ← borderline
- At 43A stall: V = (43/8450) × 1000 = **5.09V** ← dangerous

**Solution: 10kΩ/10kΩ voltage divider (halves the voltage)**

```
IBT-2 R_IS ── 10kΩ ──┬── Teensy pin 21 (A7)
                      │
                    10kΩ
                      │
                     GND
```

- Divides IS voltage by 2
- At 43A stall: Teensy sees 2.55V ✓ safe
- Adequate resolution for stall detection (not precision current measurement)

Repeat same circuit for L_IS on pin 22 (A8).

---

## Stall Detection Logic (for firmware)

```
If (PWM > 0) AND (pot reading unchanged for > 2 seconds):
    → Motor is stalled
    → Cut PWM immediately
    → Drive R_EN or L_EN LOW (reset IBT-2 fault latch)
    → Wait 100ms
    → Release EN pin (pull-up re-enables IBT-2)
    → Log stall event to serial
    → Optionally retry at reduced PWM
```

IS threshold for stall warning (before thermal shutdown):
- Read R_IS or L_IS during operation
- If analog value > threshold for > 500ms → reduce PWM or stop
- Threshold TBD from field measurement (depends on actual sense resistor value)

---

## Gen2 Test Code

`tractor_rpi/testing/teensy41_ibt2_gen2/`

Demonstrates: EN pin control, IS monitoring, stall detection, automatic fault recovery.
**Not yet integrated into main tractor firmware.**

---

## Notes

