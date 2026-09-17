# Dual-F9P Heading A/B Test

## Purpose

We need a **reliable heading stream** for an autonomous riding mower, not a fragile 10 Hz stream.

The mower already stops when heading is invalid. That is correct for safety. The problem is that the current moving-baseline setup produces invalid or zeroed heading frames that are hard to interpret, and HPG 1.32 does not give us a clean 10 Hz RELPOSNED path.

This test compares two architectures on the same tractor, same antennas, same path, and same pass/fail rules:

- **Option A:** u-blox moving-baseline heading (`UBX-NAV-RELPOSNED`)
- **Option B:** two independent RTK rovers, heading computed on the Raspberry Pi from time-matched positions

We will choose the option that stays valid more often, fails more predictably, and keeps heading accurate enough to mow.

## What we are *not* optimizing

- Peak update rate. 5 Hz that stays valid is better than 10 Hz that drops out.
- Winning an argument with the F9P firmware.
- Changing antennas, cabling, and estimator in the same outing.

## Hardware held constant

- Same two ArduSimple ZED-F9P receivers
- Same antennas, mounts, cables, and ground planes
- Same nominal baseline: **1.06–1.10 m** fore-aft
- Rear antenna is the vehicle reference / Base-Link position
- Front antenna is ahead of the reference
- Same fixed base / external RTK source
- Same Raspberry Pi host for logging
- Same operator path for the matched runs

Do not move antennas between A and B. If a mount changes, restart the comparison.

## Success criteria for a usable heading epoch

An epoch counts as **usable** only if all of the following are true:

1. Both receivers have a valid GNSS fix.
2. Both carrier solutions are **Fixed**, unless the method under test documents a valid Float-relative case. For this A/B, require Fixed on both for a fair comparison.
3. Samples used for the vector are from the **same `iTOW`** (or interpolated only when both sides remain Fixed and the time gap is ≤ 20 ms).
4. Measured 3D antenna separation is within **±5 cm** of the parked survey baseline.
5. Heading did not jump more than **8°** from the previous usable epoch at 5–10 Hz while the machine is moving slowly, or more than **3°** while parked.
6. Reported / computed heading accuracy, if available, is ≤ **2.0°**.

If any check fails, the epoch is **unusable**. The controller may hold the last good heading for a short window, but that hold does not count as a usable GNSS heading epoch.

## Shared parked survey (do this once, before A or B)

Park in the open, engine at normal idle, blades off if vibration is severe.

1. Measure tape baseline and record it.
2. Average 60 s of dual-antenna separation from the method that is currently working.
3. Record:
   - surveyed length \(L_0\)
   - heading while parked, magnetic or map, if known
   - mean C/N0 and satellites used on both receivers
4. Use \(L_0\) as the baseline gate for both options.
5. Photograph antenna placement and sky view.

Do not proceed if the two receivers disagree on length by more than ~3 cm in the open while both are Fixed.

---

## Option A — Moving baseline RELPOSNED

### Goal

See whether a **supported 5 Hz moving-baseline** configuration produces a heading stream we can live with, once we stop asking HPG 1.32 for 10 Hz.

### Configuration rules

- Firmware: HPG 1.32 on both.
- Navigation rate: **5 Hz** on both. Do not test 10 Hz in this option.
- Rear / Base-Link:
  - RTK rover from the fixed base on USB
  - moving-base RTCM out UART1
  - recommended RTCM set: `4072.0` + MSM4 (`1074/1084/1094/1124`) + `1230`
  - `4072.1` off
- Front / Heading:
  - RTCM3 in on UART1
  - `UBX-NAV-RELPOSNED` out USB
  - USB NMEA off for the scored run, or at least GSV/GSA off
- Save configuration to **flash**, power-cycle, and verify with the Pi service **stopped** before the scored run.
- Preferred starting point: ArduSimple official FW 1.32 5 Hz heading-kit files, then only the message-set trims above.

Change one thing if A is still bad after the first scored run: UART1 baud 115200 → 460800, if the wiring supports it. Do not change rate and baud in the same first run.

### What to log

- Front: `UBX-NAV-RELPOSNED` at 5 Hz
- Rear: `UBX-NAV-PVT` or `UBX-NAV-HPPOSLLH`
- Both: `UBX-RXM-RTCM`, `UBX-MON-COMMS` or TX/RX buffer messages if available
- Host timestamps and any controller “heading invalid / stop” events

### Usable-epoch mapping for A

Usable if:

- `relPosValid = true`
- `headValid = true`
- `isMoving = true`
- carrier Fixed
- baseline within ±5 cm of \(L_0\)
- baseline and accuracies not zeroed

A frame with carrier Fixed, baseline 0.0 m, `relPosValid=false`, `isMoving=false` is unusable. Count it separately. That is the failure mode we already know.

---

## Option B — Dual independent RTK + Pi heading

### Goal

See whether two identical rover configurations plus a Pi vector are more available and easier to police than moving-baseline.

### Configuration rules

- Firmware: HPG 1.32 on both.
- Both receivers configured **identically** as RTK rovers.
- Navigation rate: **10 Hz** is allowed here, because this is ordinary PVT, not moving-baseline. If we later compare apples-to-apples with A, also downsample B to 5 Hz in analysis.
- No UART interconnect. Disconnect or ignore Base-Link TX → Heading RX.
- Pi fans out the **same** fixed-base RTCM to both USB ports with minimal skew.
- Output from each receiver: `UBX-NAV-PVT` and/or `UBX-NAV-HPPOSLLH` on USB. Use high-precision fields. Do not use 7-digit NMEA lat/lon for the vector.
- Disable unused NMEA, especially GSV.

### Pi heading algorithm

For matched epochs \(r\) (rear) and \(f\) (front):

1. Pair by `iTOW`. Drop the pair if either sample is missing.
2. Convert both high-precision lat/lon/height to ECEF, then to a local ENU frame at the rear antenna.
3. Compute:

\[
\psi = \operatorname{atan2}(e_f - e_r,\ n_f - n_r)
\]

\[
L = \sqrt{(e_f-e_r)^2+(n_f-n_r)^2+(d_f-d_r)^2}
\]

4. Accept only if both are RTK Fixed and \(|L - L_0| \le 0.05\,\mathrm{m}\).
5. Do not blend “latest rear” with “latest front.”

Optional analysis-only hold: if the last usable heading is newer than 300 ms, the controller may hold it. Score held epochs as **held**, not usable.

### What to log

- Both: `UBX-NAV-PVT` / `NAV-HPPOSLLH` with `iTOW`, fix flags, carrier state, accuracy, lat/lon/height
- RTCM age on both
- Computed \(\psi\), \(L\), time skew, and gate results
- Controller stop events

---

## Test sites and runs

Run A and B on the **same day** if possible, same satellite geometry window.

### Site 1 — Clear sky

Parked 120 s, then a slow out-and-back, then a few figure-eights or tight turns at mowing speed.

### Site 2 — Operational canopy

The same parked + path pattern under the trees that already cause Float / invalid frames.

Minimum scored duration per option per site:

- 120 s parked
- 5 minutes moving

Longer is better. Keep notes on engine RPM, nearby metal, and whether the deck is running.

### Run order

1. Shared parked survey
2. Site 1 Option A
3. Site 1 Option B
4. Site 2 Option A
5. Site 2 Option B

If time is short, do Site 2 first. That is the site that matters.

Do not retune mid-run. If a config mistake is found, discard that run and repeat the whole option at that site.

---

## Metrics

Compute these the same way for A and B. Downsample B to 5 Hz when comparing rates against A, and also report B at its native rate.

### Availability

- Usable heading percentage = usable epochs / total expected epochs
- Mean time between unusable gaps
- 95th and 99th percentile gap length
- Number of controller stops
- Time from power-up to first usable heading

### Integrity

- RMS of \(L - L_0\) during usable epochs
- Count of “Fixed flags but \(L\) impossible” events
- Count of Option A specific poison frames: Fixed + baseline 0 + `isMoving=false`
- Count of Option B specific poison frames: mismatched `iTOW`, one Fixed / one Float, or large \(L\) error

### Accuracy

No independent truth sensor is assumed. Use these proxies:

- Parked heading standard deviation over 120 s
- Parked heading peak-to-peak
- Reciprocal-path heading agreement: outbound vs inbound on the same straight, after 180°
- Cross-option agreement in the open: when both would have been usable, median absolute difference A vs B
- Turn consistency: heading change during a slow 360° vs time integral of expected yaw if a yaw-rate source exists later

Open-sky parked σ should be well under **1°** on this baseline if the method is healthy. A parked σ of several degrees means the vector is not usable for autonomous work.

### Dynamics and latency

- Median inter-epoch interval
- Max inter-epoch interval
- Age of differential corrections
- For A only: RELPOSNED rate vs configured 5 Hz
- For B only: max `|iTOW_front - iTOW_rear|` on accepted pairs

### Operational score

Weight these for the decision, not raw beauty of the plot:

| Metric | Why it matters | Suggested weight |
| --- | --- | --- |
| Usable % under trees | This is the real workplace | High |
| Controller stops | Safety nuisance vs safety good | High |
| Parked heading stability | Low-speed mowing lives here | High |
| Predictability of failures | Can the Pi explain the drop? | High |
| Open-sky agreement A vs B | Detects a broken implementation | Medium |
| \(L\) stability | Integrity of the antenna vector | Medium |
| Update rate | Tie-breaker only | Low |

---

## How to choose

Use this order. Do not pick the method with the prettier open-sky plot if it loses under trees.

1. **Reject a method** if open-sky parked heading σ > 1.5° or baseline gate fails more than a few percent in the open.
2. **Prefer the method** with higher usable % under trees, provided open-sky accuracy is acceptable.
3. If usable % is similar, prefer the method with fewer unexplained poison frames and fewer controller stops.
4. If B is within ~1° of A in the open and stops less often under trees, choose **B**.
5. If A remains valid through canopy holes where B loses a fix on one antenna, choose **A** and keep it at 5 Hz.
6. If both fail under trees often enough to stop the mower, do not pick a winner. That result means we need a third source (IMU / wheel yaw) to coast through GNSS holes. The A/B still tells us which GNSS heading to fuse.

### Pre-declared thresholds

These are starting gates, not physics.

- Open-sky usable heading: ≥ **95%**
- Canopy usable heading: ≥ **80%** is encouraging; < **50%** is not a standalone GNSS heading solution
- Parked heading σ: ≤ **0.8°** good, ≤ **1.5°** acceptable, > **1.5°** fail
- Open-sky median \|A − B\| when both valid: ≤ **1.0°** expected
- Baseline gate trips in the open: < **2%** of epochs

Revise the canopy percentage after the first day if the tree site is harsher than expected, but revise both methods with the same number.

---

## Analysis notes

- Score parked and moving separately.
- Score Site 1 and Site 2 separately, then combined.
- A 300 ms hold can make a mower drive through a hole. Report raw usable % and held-usable % as two columns so we do not hide dropouts.
- A method that outputs 10 Hz with half the samples rejected is not a 10 Hz heading source.
- If Option B looks worse only because we compared 10 Hz rejects against 5 Hz A, downsample B and compare again.

## After the A/B

- If A wins: freeze the 5 Hz moving-baseline config in flash, add a Pi-side interpreter that treats zero-baseline frames as “no heading,” and write the ArduSimple letter around reliability rather than 10 Hz.
- If B wins: delete the moving-baseline UART path from the production config, keep both boards as copies of one rover file, and put the baseline gate in the controller.
- If neither wins under trees: keep the better GNSS heading from this test as the aiding source and add an IMU yaw-rate coast. Do not add the IMU until this A/B exists. Otherwise we will not know whether the IMU is covering a bad GNSS design or a real canopy hole.

## Log checklist

For every scored run record:

- Option, site, start/stop UTC
- Firmware, nav rate, baud, RTCM set
- Config hash or filename and flash-save confirmation
- Weather / canopy notes
- Raw UBX files
- CSV of usable flag, heading, \(L\), fix type, RTCM age
- Count of controller stops