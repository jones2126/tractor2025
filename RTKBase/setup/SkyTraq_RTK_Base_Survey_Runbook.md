# SkyTraq RTK Base Survey and Static-Base Commit Runbook

**Project:** Autonomous Lawn Tractor  
**Applies to:** SkyTraq PX1172RH and PX1125R RTK receivers  
**Platform in examples:** Ubuntu 24.04, `/dev/gps` or `/dev/ttyUSB0`, 115200 baud  
**Repository path:** `tractor2025/RTKBase/setup/`  
**Revision:** 2026-06-29

---

## 1. Purpose and safe workflow

This runbook documents the two-stage workflow used to establish a SkyTraq RTK base coordinate:

1. **Survey stage — SRAM only**
   - The survey script puts the receiver into **RTK Base + Survey** for a selected duration.
   - It waits until the receiver reports **run-time Static**.
   - It collects three repeat RTCM 1005 messages, decodes the coordinate, records the raw serial capture, and writes a JSON **candidate** file.
   - It does **not** write the coordinate to receiver Flash.

2. **Commit stage — SRAM + Flash**
   - The commit script reads the reviewed JSON candidate.
   - It first proves the receiver is still holding the same completed survey coordinate in run-time Static.
   - Only then does it write **RTK Base + Static** and the coordinate to SRAM and Flash.
   - After a manual power cycle, the verify mode checks that the fixed coordinate persisted.

A short survey proves the program, serial link, receiver mode transition, and RTCM output. It does **not** establish a permanent reference coordinate. A permanent coordinate should be created only with the final antenna fixed in its final outdoor location, with clear sky and low multipath.

---

## 2. Files and their roles

All files live in the repository at:

```text
tractor2025/RTKBase/setup/
```

| File | Purpose |
|---|---|
| `skytraq_rtk_base_survey_step_1.py` | Main survey program. Runs a survey in SRAM only, watches status, captures RTCM, and creates a candidate JSON file. |
| `skytraq_rtk_base_commit_step_2.py` | Review, commit, and post-power-cycle verify tool. Works with the generic candidate JSON produced for either PX1125R or PX1172RH. |
| `skytraq_rtk_base_survey.sh` | Interactive shell script. Prompts for receiver type (PX1172RH or PX1125R) and survey duration (5-minute test or 24-hour candidate), then launches `skytraq_rtk_base_survey_step_1.py` with the correct arguments. |
| `skytraq_survey_*.bin` | Raw serial capture from the survey run. Keep it with the JSON record. Not stored in the repository. |
| `skytraq_survey_candidate_*.json` | Candidate record used by the commit script. Not stored in the repository. |

### Port configuration

Both Python scripts currently use these settings near the top of each file:

```python
PORT = "/dev/gps"
BAUD = 115200
```

Change `PORT` in **both** scripts if the receiver appears on a different device path. Common alternatives are `/dev/ttyUSB0` or `/dev/ttyUSB1`. Confirm with `ls /dev/tty*` before running.

---

## 3. Key learnings from the testing

### 3.1 What proves a survey succeeded

A completed survey requires all of the following:

```text
RTK mode:                 Base
Saved operational state:  Survey
Run-time operational:     Static
Run-time survey length:   0
Three repeat RTCM 1005 messages agree
```

The script intentionally does not write a JSON candidate if it does not receive this evidence.

### 3.2 RTCM 1005 is the coordinate record used by the workflow

RTCM message 1005 carries the base position as ECEF X/Y/Z coordinates. The survey script converts those values into:

```text
Latitude
Longitude
WGS-84 ellipsoid height
ECEF X/Y/Z
```

**Use `ellipsoid_height_m` for all base-coordinate work in these scripts.** It is not mean-sea-level elevation and is not a generic "altitude" field.

### 3.3 The JSON candidate is a handoff record, not a Flash write

A candidate JSON file contains:

- the survey coordinate;
- source method and survey duration;
- raw-capture file name and path;
- confirmation that three post-static 1005 frames matched;
- receiver serial settings;
- the RTCM message types and counts observed;
- an audit log.

It begins with:

```json
"candidate_status": "not_committed"
```

Do not edit its coordinate values by hand.

### 3.4 The successful indoor/window tests are functional tests only

The PX1172RH test output showed the full successful sequence:

```text
run-time Static
three matching RTCM 1005 frames
JSON candidate written
```

That proves the software workflow works. The antenna was indoors near a window, which is not suitable for a permanent coordinate because reflections, blocked sky, and changing satellite geometry can move a short-survey result by meters.

### 3.5 Do not power-cycle between a successful candidate survey and its commit

The commit script deliberately requires the receiver to be holding the same completed survey coordinate in run-time Static. This protects against committing the wrong JSON record.

Therefore:

```text
Run successful survey
→ review candidate JSON
→ preview candidate
→ commit that same live candidate
→ power-cycle
→ verify
```

A power cycle before `--commit` will normally make the commit script refuse to proceed. That is a safety feature.

---

## 4. Preflight checklist

Before every survey or commit:

1. Close GNSS Viewer.
2. Stop any Python service, NTRIP forwarder, or other program using the receiver serial port.
3. Do not forward the mixed raw serial stream to a rover while surveying or committing.
4. Confirm the receiver has stable power for the whole run.
5. Confirm `PORT` and `BAUD` in both Python scripts.
6. Ensure the computer will not sleep during the survey.
7. Ensure the working folder has adequate free space for the `.bin` raw capture.
8. For a permanent base, mount the antenna permanently before beginning the long survey. Do not move the antenna, cable, or receiver during the run.

For the PX1172RH with one antenna, connect the antenna to **AJ2 / RF_IN1**. AJ1 is the second heading-antenna input and did not provide a one-antenna navigation solution in testing.

---

## 5. Main survey script behavior

The current survey script is:

```text
skytraq_rtk_base_survey_step_1.py
```

It always uses **SRAM only** during the survey. It does not write Flash.

### Standard monitor behavior

In normal mode, it:

1. Queries the current RTK state.
2. Starts RTK Base + Survey.
3. Polls status at a chosen interval.
4. Watches for run-time Static.
5. For PX1172RH-style countdown behavior, aborts if the run-time survey length does not reduce for a configured number of polls.
6. Collects three matching post-static RTCM 1005 messages.
7. Writes a raw `.bin` file and a candidate `.json` file.

The general timeout is:

```text
max(survey duration + post-survey grace, minimum monitor timeout)
```

---

## 6. Use case A — short five-minute functional test

A five-minute survey is useful to prove:

- serial communication;
- Survey → Static transition;
- correct RTCM output;
- JSON candidate creation;
- the later commit script's preview behavior.

It is **not** the coordinate to commit as a permanent base.

### Interactive shell script (recommended)

The easiest way to run any survey is with the interactive script. Make it executable once after cloning:

```bash
chmod +x skytraq_rtk_base_survey.sh
```

Then run it:

```bash
./skytraq_rtk_base_survey.sh
```

It will prompt for receiver type and survey duration, then launch the correct command automatically.

### PX1172RH: five-minute test (manual command)

Use the progress-monitor script directly. It checks the PX1172RH countdown every 20 seconds and permits a 15-minute total monitor window.

```bash
python3 skytraq_rtk_base_survey_step_1.py \
  --duration 300 \
  --label px1172rh-5min-test \
  --status-poll-seconds 20 \
  --post-survey-grace-seconds 300 \
  --minimum-monitor-timeout-seconds 900 \
  --max-stalled-polls 3 \
  --max-missing-status-polls 3
```

Expected successful ending:

```text
Run-time operational:    Static
Run-time survey length:  0 second(s)
COLLECTING 3 REPEAT RTCM 1005 MESSAGES
JSON survey candidate written: ...json
```

### PX1125R: five-minute test (manual command)

Use the same script, but do **not** rely on early countdown-stall detection. During earlier PX1125R testing, its run-time survey-length field did not demonstrate the same reliably decrementing behavior observed on the PX1172RH.

```bash
python3 skytraq_rtk_base_survey_step_1.py \
  --duration 300 \
  --label px1125r-5min-test \
  --status-poll-seconds 20 \
  --post-survey-grace-seconds 300 \
  --minimum-monitor-timeout-seconds 900 \
  --max-stalled-polls 100000 \
  --max-missing-status-polls 3
```

`--max-stalled-polls 100000` effectively disables the early "countdown did not reduce" abort while retaining normal status checks, the overall timeout, and missing-status protection.

---

## 7. Use case B — 24-hour permanent-base candidate survey

A 24-hour survey should be run only after the final base antenna is permanently mounted outdoors with clear sky, good ground plane, stable power, and no nearby reflective metal, walls, windows, trees, or roof obstructions.

A 24-hour candidate is still not automatically permanent. Review it and, ideally, compare it with another independent long run before committing.

### PX1172RH: 24-hour candidate (manual command)

This polls every five minutes. It allows an additional one-hour grace period beyond the 24-hour requested duration. Three unchanged/increasing countdown readings in a row produce an early abort after approximately 15 minutes of no observed progress.

```bash
python3 skytraq_rtk_base_survey_step_1.py \
  --duration 86400 \
  --label px1172rh-permanent-base-24h \
  --status-poll-seconds 300 \
  --post-survey-grace-seconds 3600 \
  --minimum-monitor-timeout-seconds 90000 \
  --max-stalled-polls 3 \
  --max-missing-status-polls 3
```

### PX1125R: 24-hour candidate (manual command)

Use the same overall timeout and five-minute polling interval, but disable early countdown-stall termination because that specific countdown behavior has not been validated on this receiver.

```bash
python3 skytraq_rtk_base_survey_step_1.py \
  --duration 86400 \
  --label px1125r-permanent-base-24h \
  --status-poll-seconds 300 \
  --post-survey-grace-seconds 3600 \
  --minimum-monitor-timeout-seconds 90000 \
  --max-stalled-polls 100000 \
  --max-missing-status-polls 3
```

### Compare a second survey with the first

After completing an initial long candidate, run a second survey at the same fixed antenna reference point and add `--compare` with the first candidate file:

```bash
python3 skytraq_rtk_base_survey_step_1.py \
  --duration 86400 \
  --label px1172rh-permanent-base-24h-run2 \
  --status-poll-seconds 300 \
  --post-survey-grace-seconds 3600 \
  --minimum-monitor-timeout-seconds 90000 \
  --max-stalled-polls 3 \
  --compare "skytraq_survey_candidate_YYYYMMDD_HHMMSS_px1172rh-permanent-base-24h.json"
```

The new JSON record and console output will include East/North/Up, horizontal distance, and 3D distance relative to the prior candidate.

Investigate any multi-meter disagreement before selecting a permanent coordinate. Do not expect an indoor/window test to agree closely with an outdoor permanent-base survey.

---

## 8. How survey output is used by the next script

The survey script produces two primary outputs:

```text
skytraq_survey_YYYYMMDD_HHMMSS_<label>.bin
skytraq_survey_candidate_YYYYMMDD_HHMMSS_<label>.json
```

The `.bin` file is the raw evidence capture. Preserve it but do not feed it directly into the commit script.

The `.json` candidate is the input to:

```text
skytraq_rtk_base_commit_step_2.py
```

### Step 1 — Preview only: review the candidate with no receiver change

```bash
python3 skytraq_rtk_base_commit_step_2.py --config "skytraq_survey_candidate_YYYYMMDD_HHMMSS_<label>.json"
```

This mode sends no command to the receiver. It displays the target coordinate from the JSON file.

### Step 2 — Commit the reviewed candidate to SRAM and Flash

Only run this while the receiver is still in the completed Survey / run-time Static state produced by that exact candidate.

```bash
python3 skytraq_rtk_base_commit_step_2.py \
  --config "skytraq_survey_candidate_YYYYMMDD_HHMMSS_<label>.json" \
  --commit
```

The script asks for the exact confirmation phrase:

```text
COMMIT STATIC BASE
```

Before it writes Flash, it checks:

```text
Receiver is RTK Base
Saved mode is Survey
Run-time mode is Static
Run-time coordinate closely agrees with the JSON target
```

After the Flash write, it checks that the receiver reports **saved Static**, **run-time Static**, and that repeat RTCM 1005 coordinates agree with the JSON target.

### Step 3 — Manual power-cycle and persistence verification

After a successful commit, power-cycle the receiver. Then, with GNSS Viewer closed and no serial forwarder running:

```bash
python3 skytraq_rtk_base_commit_step_2.py \
  --config "skytraq_survey_candidate_YYYYMMDD_HHMMSS_<label>.json" \
  --verify
```

A successful verify appends a persistence-verification entry to the candidate JSON's audit log and writes a `.bak` backup of the JSON before modifying it.

---

## 9. PX1172RH versus PX1125R: operational differences

| Topic | PX1172RH | PX1125R |
|---|---|---|
| One-antenna connection | Use **AJ2 / RF_IN1**. AJ1 / RF_IN2 is the second heading-antenna input. | Use the board's normal primary GNSS antenna input. |
| Receiver role | Dual-antenna position-and-heading receiver; one antenna is sufficient for a position-only base test when connected to AJ2. | Single-antenna RTK base/rover-oriented receiver. |
| Survey command behavior observed | GNSS Viewer carried a prior run-time Static coordinate in the survey command's coordinate fields. The current progress-monitor script reproduces that behavior automatically when a valid prior Static coordinate is available. | A previous PX1125R survey succeeded with zero coordinate fields in Survey mode. The current script can still carry a valid prior Static coordinate; the protocol treats the coordinate fields as unused in Survey mode. |
| Run-time survey length | Observed to decrement, but wall-clock completion was slower than the requested 60 seconds in indoor/window testing. Use the progress monitor and its stall detection. | Did not demonstrate a reliably monotonic countdown during prior testing. Do not use countdown non-progress alone as a PX1125R failure criterion. |
| Recommended short-test monitor | 20-second polls; 3 stalled polls allowed. | 20-second polls; set `--max-stalled-polls 100000`. |
| Recommended 24-hour monitor | 5-minute polls; 3 stalled polls allowed; one-hour grace. | 5-minute polls; disable early countdown-stall abort; one-hour grace. |
| Commit/verify tool | `skytraq_rtk_base_commit_step_2.py` | `skytraq_rtk_base_commit_step_2.py` |

---

## 10. Expected RTCM output

A useful multi-constellation base stream commonly includes:

```text
1005  reference-station coordinate
1074  GPS MSM4 observations
1084  GLONASS MSM4 observations
1094  Galileo MSM4 observations
1124  BeiDou MSM4 observations
1230  GLONASS code-phase biases
```

Broadcast ephemeris and metadata messages such as 1019, 1020, 1033, 1042, and 1046 may also appear.

Message availability depends on receiver configuration, antenna reception, and satellite visibility. The core survey success test in these scripts is the receiver's run-time Static state plus three matching RTCM 1005 messages.

---

## 11. Troubleshooting guide

### Survey remains in Survey mode

- Confirm GNSS Viewer and all other serial consumers are closed.
- Confirm correct device path and 115200 baud in the Python script.
- Confirm the antenna is connected to PX1172RH AJ2 for one-antenna testing.
- Check outdoor/clear-sky reception before treating the condition as a script failure.
- For PX1172RH, start from run-time Static when possible so the script can mirror GNSS Viewer's previously observed survey packet behavior.
- Use the PX1172RH progress-monitor defaults before using diagnostic mode.

### "Run-time survey length did not reduce" abort

- On PX1172RH, this suggests the receiver is not showing survey progress across the configured polls. Inspect power, antenna, sky view, and any unexpected application using the port.
- On PX1125R, use the PX1125R command examples with `--max-stalled-polls 100000` because the countdown has not been demonstrated as a reliable monitor signal.

### No JSON candidate written

The script did not see confirmed run-time Static and three matching 1005 frames. It did not write Flash. Review the console output and raw `.bin` capture.

### Commit script refuses to write Flash

This is intentional if the receiver's current live coordinate does not agree very closely with the chosen JSON candidate. The usual causes are:

- a power cycle occurred after the survey;
- a different survey was run after the candidate was created;
- the wrong candidate JSON file was selected;
- the antenna or receiver was moved.

Re-run the chosen survey and commit its matching candidate while it is still live in run-time Static.

### Permission denied on serial port

Add the user to the `dialout` group if not already a member:

```bash
sudo usermod -aG dialout al
```

Log out and back in for the change to take effect. Confirm access with:

```bash
ls -l /dev/gps
```

---

## 12. Current status

The PX1172RH workflow has now been demonstrated end-to-end through candidate creation:

```text
PX1172RH on AJ2
→ Base Survey
→ monitored survey countdown
→ run-time Static
→ three matching RTCM 1005 frames
→ raw serial capture
→ JSON candidate file
```

The current remaining steps for a real base are not programming changes. They are field procedure:

```text
final outdoor antenna location
→ long candidate survey
→ independent comparison survey(s)
→ review selected candidate
→ commit without an intervening reboot
→ manual power-cycle
→ verify
```
