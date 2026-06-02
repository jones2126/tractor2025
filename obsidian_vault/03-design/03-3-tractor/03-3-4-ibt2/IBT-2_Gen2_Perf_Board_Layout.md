# IBT-2 Gen2 — Perf Board Layout & Assembly

**Board size:** 2" × 4" landscape (4" wide, 2" tall)  
**Grid:** 0.1" hole spacing — 40 columns × 20 rows  
**Origin (0,0):** lower-left corner  
**Coordinates:** (x, y) in inches — x increases right, y increases up

See also: [[IBT-2_Gen2_Build_Guide]] for circuit theory and component list.

---

## Grid Reference

```
y=1.9  row 19  ← top edge
y=1.8  row 18
  ...
y=0.1  row  1
y=0.0  row  0  ← bottom edge

x=0.0  col  0  ← left edge
x=0.1  col  1
  ...
x=3.8  col 38
x=3.9  col 39  ← right edge
```

One hole = 0.1". Adjacent holes are 0.1" apart. A standard 1/4W resistor lying flat
spans **4 holes (0.4")** between leads. 100nF ceramic disc leads can be bent to any
spacing; this layout uses 4 holes (0.4") matching the resistors beside them. The 100µF
electrolytic is placed vertically with a 2-hole (0.2") lead spacing.

---

## Power Rails

Run a bare tinned copper wire solder-bridged along each rail before placing any components.

| Rail | Row | y (in) | Extent |
|------|-----|--------|--------|
| **GND bus** | 0 | 0.0 | col 0 → col 39 |
| **5V bus**  | 19 | 1.9 | col 0 → col 39 |

---

## Board Sections (left to right)

```
x=0.0–0.3   IBT-2 signal inputs     (col 0–3)
x=0.3–0.5   wire routing gap
x=0.5–0.9   EN pull-up resistors R1, R2
             IS upper divider resistors R3, R5
x=0.9–1.2   IS lower dividers R4, R6 + filter caps C1, C2
x=1.2–2.1   open routing area
x=2.2       VCC bypass cap C3
x=2.2–3.1   open routing area
x=3.3       DROK power input connector
x=3.5–3.9   Teensy signal outputs   (col 35–39)
```

---

## Component Placement

All components placed with leads in the holes listed. Lie resistors flat (body
horizontal); stand caps vertically. No component shares a hole with another.

### R1 — R_EN pull-up (10kΩ, horizontal, row 16)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — 5V end | 0.5 | 1.6 | 5 | 16 | → wire up to 5V bus |
| B — signal | 0.9 | 1.6 | 9 | 16 | R_EN_NODE |

### R2 — L_EN pull-up (10kΩ, horizontal, row 13)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — 5V end | 0.5 | 1.3 | 5 | 13 | → wire up to 5V bus |
| B — signal | 0.9 | 1.3 | 9 | 13 | L_EN_NODE |

### R3 — R_IS upper divider (10kΩ, horizontal, row 10)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — input | 0.5 | 1.0 | 5 | 10 | → wire left to R_IS input |
| B — mid   | 0.9 | 1.0 | 9 | 10 | R_IS_MID_NODE |

### R5 — L_IS upper divider (10kΩ, horizontal, row 7)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — input | 0.5 | 0.7 | 5 | 7 | → wire left to L_IS input |
| B — mid   | 0.9 | 0.7 | 9 | 7 | L_IS_MID_NODE |

### R4 — R_IS lower divider (10kΩ, vertical, col 10)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — top    | 1.0 | 1.0 | 10 | 10 | R_IS_MID_NODE — solder-bridge to R3 lead B |
| B — bottom | 1.0 | 0.6 | 10 |  6 | → wire to GND bus |

### R6 — L_IS lower divider (10kΩ, vertical, col 10)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| A — top    | 1.0 | 0.7 | 10 | 7 | L_IS_MID_NODE — solder-bridge to R5 lead B |
| B — bottom | 1.0 | 0.3 | 10 | 3 | → wire to GND bus |

> **Caution:** R4 bottom is at (1.0, 0.6) and R6 top is at (1.0, 0.7) — two adjacent
> holes in the same column. Solder carefully; do not bridge these two joints.

### C1 — R_IS noise filter (100nF ceramic, vertical, col 11)

No polarity — either lead orientation is fine.

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| 1 — top    | 1.1 | 1.0 | 11 | 10 | solder-bridge to R4 lead A at (1.0, 1.0) |
| 2 — bottom | 1.1 | 0.6 | 11 |  6 | solder-bridge to R4 lead B at (1.0, 0.6) |

### C2 — L_IS noise filter (100nF ceramic, vertical, col 11)

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| 1 — top    | 1.1 | 0.7 | 11 | 7 | solder-bridge to R6 lead A at (1.0, 0.7) |
| 2 — bottom | 1.1 | 0.3 | 11 | 3 | solder-bridge to R6 lead B at (1.0, 0.3) |

> **Caution:** C1 bottom is at (1.1, 0.6) and C2 top is at (1.1, 0.7) — two adjacent
> holes in the same column. Same caution as R4/R6 above.

### C3 — VCC bypass (100µF / 16V electrolytic, vertical, col 22)

Longer lead = positive (+). The stripe printed on the side marks the negative (−) lead.

| Lead | x (in) | y (in) | Col | Row | Node |
|------|--------|--------|-----|-----|------|
| + (long lead) | 2.2 | 1.9 | 22 | 19 | 5V bus — insert directly into bus row |
| − (short lead / stripe) | 2.2 | 1.7 | 22 | 17 | → wire to GND bus |

---

## Connector Positions

Use 2.54mm (0.1") pin headers, screw terminals, or bare wire pads — your choice.
All connectors run vertically in their respective columns.

### IBT-2 Signal Inputs (col 2, left side)

| x (in) | y (in) | Col | Row | Signal |
|--------|--------|-----|-----|--------|
| 0.2 | 1.9 | 2 | 19 | IBT-2 VCC (5V) — in 5V bus row |
| 0.2 | 1.6 | 2 | 16 | IBT-2 R_EN |
| 0.2 | 1.3 | 2 | 13 | IBT-2 L_EN |
| 0.2 | 1.0 | 2 | 10 | IBT-2 R_IS |
| 0.2 | 0.7 | 2 |  7 | IBT-2 L_IS |
| 0.2 | 0.4 | 2 |  4 | IBT-2 RPWM (passthrough) |
| 0.2 | 0.2 | 2 |  2 | IBT-2 LPWM (passthrough) |
| 0.2 | 0.0 | 2 |  0 | IBT-2 GND — in GND bus row |

### DROK Power Input (col 33)

| x (in) | y (in) | Col | Row | Signal |
|--------|--------|-----|-----|--------|
| 3.3 | 1.9 | 33 | 19 | DROK 5V output — in 5V bus row |
| 3.3 | 0.0 | 33 |  0 | DROK GND — in GND bus row |

### Teensy Signal Outputs (col 37, right side)

| x (in) | y (in) | Col | Row | Signal | Teensy pin |
|--------|--------|-----|-----|--------|-----------|
| 3.7 | 1.6 | 37 | 16 | R_EN | Pin 7 |
| 3.7 | 1.3 | 37 | 13 | L_EN | Pin 8 |
| 3.7 | 1.0 | 37 | 10 | R_IS | Pin 21 (A7) |
| 3.7 | 0.7 | 37 |  7 | L_IS | Pin 22 (A8) |
| 3.7 | 0.4 | 37 |  4 | RPWM | Pin 5 |
| 3.7 | 0.2 | 37 |  2 | LPWM | Pin 6 |
| 3.7 | 0.0 | 37 |  0 | GND | GND |

---

## Wiring List

Run wires on the **bottom (solder) side** of the board using 22–24 AWG insulated hookup wire.
The "solder-bridge" items are tiny blobs of solder connecting two adjacent pads — no wire needed.

### Power

| # | From | To | Length | Notes |
|---|------|----|--------|-------|
| W1 | (0.5, 1.6) R1 lead A | (0.5, 1.9) 5V bus | 0.3" | R_EN pull-up to 5V |
| W2 | (0.5, 1.3) R2 lead A | (0.5, 1.9) 5V bus | 0.6" | L_EN pull-up to 5V |
| W3 | (1.0, 0.6) R4 lead B | (1.0, 0.0) GND bus | 0.6" | R_IS lower to GND |
| W4 | (1.0, 0.3) R6 lead B | (1.2, 0.0) GND bus | ~0.4" | L_IS lower to GND — route diagonally to avoid col 10 congestion |
| W5 | (2.2, 1.7) C3 − lead | (2.2, 0.0) GND bus | 1.7" | VCC bypass cap to GND |

### Solder Bridges (adjacent holes, no wire)

| # | Hole A | Hole B | Purpose |
|---|--------|--------|---------|
| B1 | (0.9, 1.0) R3-B | (1.0, 1.0) R4-A | Joins R3 output to R4 top / R_IS_MID |
| B2 | (0.9, 0.7) R5-B | (1.0, 0.7) R6-A | Joins R5 output to R6 top / L_IS_MID |
| B3 | (1.0, 1.0) R4-A | (1.1, 1.0) C1-1 | Puts C1 in parallel with R4 (top) |
| B4 | (1.0, 0.6) R4-B | (1.1, 0.6) C1-2 | Puts C1 in parallel with R4 (bottom) |
| B5 | (1.0, 0.7) R6-A | (1.1, 0.7) C2-1 | Puts C2 in parallel with R6 (top) |
| B6 | (1.0, 0.3) R6-B | (1.1, 0.3) C2-2 | Puts C2 in parallel with R6 (bottom) |

### Signal Wires (left to right)

| # | From | To | Length | Signal |
|---|------|----|--------|--------|
| W6  | (0.2, 1.6) IBT-2 R_EN input | (0.9, 1.6) R1 lead B | 0.7" | R_EN from IBT-2 to pull-up node |
| W7  | (0.2, 1.3) IBT-2 L_EN input | (0.9, 1.3) R2 lead B | 0.7" | L_EN from IBT-2 to pull-up node |
| W8  | (0.2, 1.0) IBT-2 R_IS input | (0.5, 1.0) R3 lead A | 0.3" | R_IS from IBT-2 to upper divider |
| W9  | (0.2, 0.7) IBT-2 L_IS input | (0.5, 0.7) R5 lead A | 0.3" | L_IS from IBT-2 to upper divider |
| W10 | (0.9, 1.6) R_EN_NODE | (3.7, 1.6) Teensy pin 7 | 2.8" | R_EN out to Teensy |
| W11 | (0.9, 1.3) L_EN_NODE | (3.7, 1.3) Teensy pin 8 | 2.8" | L_EN out to Teensy |
| W12 | (0.9, 1.0) R_IS_MID  | (3.7, 1.0) Teensy pin 21 | 2.8" | R_IS out to Teensy A7 |
| W13 | (0.9, 0.7) L_IS_MID  | (3.7, 0.7) Teensy pin 22 | 2.8" | L_IS out to Teensy A8 |
| W14 | (0.2, 0.4) RPWM in | (3.7, 0.4) Teensy pin 5 | 3.5" | RPWM passthrough |
| W15 | (0.2, 0.2) LPWM in | (3.7, 0.2) Teensy pin 6 | 3.5" | LPWM passthrough |

> W10–W15 are the four long signal wires crossing the board. Run them side-by-side on
> the underside, bundled together with a small zip-tie or heat-shrink sleeve in the
> middle of the board to keep them tidy. Use different colors:
> - Red = R_EN (W10)
> - Orange = L_EN (W11)
> - Yellow = R_IS (W12)
> - Green = L_IS (W13)
> - White = RPWM (W14)
> - Blue = LPWM (W15)

---

## Board Overview Diagram

```
  x→  0.0  0.2  0.4  0.6  0.8  1.0  1.2  1.4  ...  2.2  ...  3.3  3.5  3.7  3.9
y↑
1.9  ══[VCC]═══════════[5V BUS]═══════════════════[C3+]═════[DRK5V]══════════════
1.8  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
1.7  · · · · · · · · · · · · · · · · · · · · · ·[C3-]· · · · · · · · · · · · · ·
1.6  [R_EN]·──────[===R1===]──────────────────────────────────────────────[T.7] ·
1.5  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
1.4  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
1.3  [L_EN]·──────[===R2===]──────────────────────────────────────────────[T.8] ·
1.2  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
1.1  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
1.0  [R_IS]·──[===R3===]─┬[R4 ╔C1]────────────────────────────────────────[A7] ·
0.9  · · · · · · · · · · │   ║ ║ · · · · · · · · · · · · · · · · · · · · · · · ·
0.8  · · · · · · · · · · │   ║ ║ · · · · · · · · · · · · · · · · · · · · · · · ·
0.7  [L_IS]·──[===R5===]─┴[R6 ╔C2]────────────────────────────────────────[A8] ·
0.6  · · · · · · · · · ·  ╚══╝ ║ · · · · · · · · · · · · · · · · · · · · · · · ·
0.5  · · · · · · · · · · · · · ║ · · · · · · · · · · · · · · · · · · · · · · · ·
0.4  [RPWM]·············────────────────────────────────────────────────── [P5] ·
0.3  · · · · · · · · · · · ╚══╝ · · · · · · · · · · · · · · · · · · · · · · · ·
0.2  [LPWM]·············─────────────────────────────────────────────────── [P6]·
0.1  · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·
0.0  ══[GND]═══════════[GND BUS]══════════════════════════[DRKGND]═════════[GND]═
```

Legend:
- `[===Rx===]` resistor body lying flat
- `[Rx` / `Cx]` vertical component (shown as column, top to bottom)
- `╔` `╚` `╝` solder-bridge between adjacent columns
- `──` wire (on bottom/solder side of board)

---

## Assembly Order

Follow this sequence — it's easier to solder components before adding wires.

1. **Power rails first:** Solder a bare tinned wire along row 0 (GND) and row 19 (5V), col 0→39.

2. **Connectors:** Solder pin headers or bare wire pads at:
   - Col 2 (IBT-2 side): rows 19, 16, 13, 10, 7, 4, 2, 0
   - Col 33 (DROK power): rows 19, 0
   - Col 37 (Teensy side): rows 16, 13, 10, 7, 4, 2, 0

3. **Resistors (all 10kΩ, lie flat):**
   - R1: (0.5, 1.6) → (0.9, 1.6)
   - R2: (0.5, 1.3) → (0.9, 1.3)
   - R3: (0.5, 1.0) → (0.9, 1.0)
   - R5: (0.5, 0.7) → (0.9, 0.7)
   - R4: (1.0, 1.0) → (1.0, 0.6) — stand vertically
   - R6: (1.0, 0.7) → (1.0, 0.3) — stand vertically

4. **Ceramic caps (100nF, stand vertically, no polarity):**
   - C1: (1.1, 1.0) → (1.1, 0.6)
   - C2: (1.1, 0.7) → (1.1, 0.3)

5. **Electrolytic cap (100µF / 16V — observe polarity!):**
   - C3: + at (2.2, 1.9), − at (2.2, 1.7)

6. **Solder bridges B1–B6** (tiny blobs connecting adjacent pads).

7. **Power wires W1–W5** (short vertical/diagonal runs to buses).

8. **Signal wires W6–W9** (short, left section).

9. **Long signal wires W10–W13** (R_EN, L_EN, R_IS, L_IS across the board).

10. **Passthrough wires W14–W15** (RPWM, LPWM across full board length).

---

## Pre-Power Checks

Before connecting to the Teensy or IBT-2:

- [ ] Continuity: 5V bus pin at (0.2, 1.9) → C3 + at (2.2, 1.9) → DROK input at (3.3, 1.9)
- [ ] Continuity: GND bus pin at (0.2, 0.0) → R4 bottom via W3 → C3 − via W5
- [ ] No short: measure resistance between 5V bus and GND bus — should read >100kΩ before any power is applied
- [ ] C3 polarity: verify longer lead is at (2.2, 1.9) — on the 5V bus
- [ ] R4 / R6 not bridged: probe (1.0, 0.6) vs (1.0, 0.7) — should be isolated (no continuity)
- [ ] C1 / C2 not bridged: probe (1.1, 0.6) vs (1.1, 0.7) — should be isolated

---

## Parts to Order

| Qty | Part | Notes |
|-----|------|-------|
| 6 | 10kΩ 1/4W resistor | All same value — any brand |
| 2 | 100nF ceramic disc cap | Marked "104" — 25V or higher rating |
| 1 | 100µF / 16V electrolytic cap | Radial, 0.2" (5mm) lead spacing |
| 1 | 2" × 4" perf board | 0.1" hole pitch, non-copper-clad (bare perf) or copper-clad single-side |
| 2 | 8-pin 0.1" male header strip | IBT-2 and Teensy connectors |
| 1 | 2-pin screw terminal 3.5mm | DROK power input |
| 1 | Small spool 22–24 AWG hookup wire | Assorted colors — 6 colors ideal |
| 1 | Spool of bare tinned 22 AWG wire | For bus rails |
