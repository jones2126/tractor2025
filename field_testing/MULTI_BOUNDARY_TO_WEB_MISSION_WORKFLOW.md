# Multi-boundary capture to web-launched master mission

This is the repeatable workflow developed from the 62 Collins survey on
2026-09-15. The next intended use is 70 Collins Drive.

The process has five distinct products. Do not treat them as interchangeable:

1. **Raw capture** — the complete `field_test_*.csv` written on the tractor.
2. **Candidate segments** — Pause-separated paths extracted from the raw CSV.
3. **Reviewed segment index** — human labels identifying boundaries,
   transitions, obstacles, and rejected runs.
4. **Master mission package** — generated waypoints, audit index, report, map,
   launcher, and recovery validation.
5. **Web dashboard integration** — the reviewed mission and launcher selected
   by the real-time monitoring interface.

The raw capture is always the authoritative source. Candidate segment CSVs and
mission files can be regenerated from it.

## Required project context

- Tractor repository on the robot: `/home/al/tractor2025`
- Tractor ZeroTier address: `192.168.193.76`
- RTK base ZeroTier address: `192.168.193.88`
- Expected tractor firmware for the 62 Collins workflow:
  `teensy_main_20260914`
- Field logger: `tractor_rpi/field_test_logger_20260828.py`
- Mission preflight: `tractor_rpi/testing/mission_preflight_20260804.py`
- Guarded Pure Pursuit controller:
  `tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py`
- Web dashboard:
  `tractor_rpi/pure-pursuit/mission_dashboard_20260910.py`
- GPS feeds: logger `6009`, navigation `6010`, dashboard positioning `6013`
- Dashboard control and telemetry: `6011` and `6012`
- Teensy bridge status and commands: `6003` and `6004`

Before using this workflow at 70 Collins, confirm the current firmware and
port assignments still match these values. Update the new launcher if they do
not.

## 1. Plan the manual survey

Create a written drive order before moving the tractor. Every independently
labelled item should be one Manual-mode drive bracketed by at least five
seconds in Pause.

Recommended segment types:

- `transition` — a deliberately driven safe connector between areas;
- `polygon-<name>` — one continuous closed outer boundary;
- `obstacle` — a separate closed loop around one obstacle;
- `no` — departure, repositioning, mistake, or incomplete run to exclude.

For a cleaner 70 Collins capture:

- Drive each outer boundary as one closed loop when physically possible.
- Pause for at least five seconds before and after every boundary.
- Record obstacle loops as separate segments after completing the outer
  boundary. Avoid inserting an obstacle loop into the middle of an outer
  boundary unless access makes that unavoidable.
- Deliberately drive every connector that the autonomous mission may later
  use. These recorded connectors are preferred because they reflect the
  operator's knowledge of trees, slopes, pavement, gates, and other hazards.
- Start and finish a closed loop at an open, repeatable location with enough
  room to enter and leave it.
- Keep the mower deck disengaged during the survey.
- If RTK Fixed is lost during a boundary, select Pause. After RTK Fixed
  returns, repeat that boundary as a new complete segment and mark the partial
  segment for exclusion.

Keep a simple field note such as:

```text
01 leave parking
02 safe transition to rear yard
03 rear-yard boundary
04 oak tree obstacle
05 transition to side yard
06 side-yard boundary
07 return to parking
```

The numbers are only the expected order. The analysis tool assigns the final
segment numbers from the captured Pause intervals.

## 2. Check the RTK system and run preflight

On the base station, verify the existing static base service rather than
starting another survey:

```bash
ssh al@192.168.193.88
systemctl is-active rtcm_server.service
journalctl -u rtcm_server.service --since '5 minutes ago' --no-pager | tail -30
exit
```

The service should be active and the recent report should show current RTCM
traffic, including Type 1005 and observation messages, with a tractor client
connected.

On the tractor, keep the handheld in Pause and the blades disengaged:

```bash
cd /home/al/tractor2025
sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260914
```

Do not begin the survey unless preflight passes, LED4 indicates RTK Fixed, and
the heading solution is valid and fixed.

## 3. Capture the continuous field telemetry

Choose an explicit file name so the source is easy to identify later:

```bash
cd /home/al/tractor2025
python3 -u tractor_rpi/field_test_logger_20260828.py --output /home/al/field_logs/70_Collins_boundary_YYYYMMDD_HHMMSS.csv
```

Capture procedure:

1. Remain in Pause for at least five seconds.
2. Select Manual and drive exactly one planned segment.
3. Select Pause and remain stopped for at least five seconds.
4. Repeat for every boundary, obstacle, and transition.
5. Press `Ctrl+C` after the final Pause interval. Wait for the logger's clean
   shutdown message and final file path.

Record and verify the tractor-side checksum:

```bash
sha256sum /home/al/field_logs/70_Collins_boundary_YYYYMMDD_HHMMSS.csv
```

## 4. Copy and verify the raw capture

Use this structure on the development machine:

```text
field_testing/sites/70_Collins_multi_boundary_YYYYMMDD/
  runs/YYYYMMDD_HHMMSS/
    field_test_YYYYMMDD_HHMMSS.csv
    boundary_review/
  mission_plans/
    YYYYMMDD_master_boundary_replay/
```

PowerShell example:

```powershell
$run = 'C:\Repos\tractor2025\field_testing\sites\70_Collins_multi_boundary_YYYYMMDD\runs\YYYYMMDD_HHMMSS'
New-Item -ItemType Directory -Force -Path $run | Out-Null
scp al@192.168.193.76:/home/al/field_logs/70_Collins_boundary_YYYYMMDD_HHMMSS.csv "$run\field_test_YYYYMMDD_HHMMSS.csv"
Get-FileHash "$run\field_test_YYYYMMDD_HHMMSS.csv" -Algorithm SHA256
```

The Windows hash must exactly match the tractor-side hash before analysis.

## 5. Create candidate segments

The reusable segmentation tool is:

`field_testing/tools/analyze_multi_boundary_capture_20260915.py`

It treats qualifying Pause intervals as separators, rejects very short drive
intervals, reduces repeated GPS positions, samples the path at approximately
0.25 m, and creates:

- `segment_NN_candidate_path.csv` for each candidate;
- `candidate_segments_overview.png`;
- `candidate_segments_summary.json`;
- `candidate_segments_map_data.json`;
- `segment_labeling_worksheet.csv`.

Run it from the repository root. PowerShell example:

```powershell
$run = 'C:\Repos\tractor2025\field_testing\sites\70_Collins_multi_boundary_YYYYMMDD\runs\YYYYMMDD_HHMMSS'
python field_testing/tools/analyze_multi_boundary_capture_20260915.py "$run\field_test_YYYYMMDD_HHMMSS.csv" --output-dir "$run\boundary_review" --title "70 Collins multi-boundary survey"
```

Default segmentation thresholds are a 3-second Pause, an 8-second drive, a
5 m minimum path, and 0.25 m candidate-point spacing. Adjust them only after
reviewing why a deliberate segment was missed or why noise was included.

### Prompt to request segmentation and a review map

Use this prompt after the verified CSV is in the repository:

```text
Analyze the 70 Collins multi-boundary capture at:
<absolute path to field_test_YYYYMMDD_HHMMSS.csv>

Use field_testing/tools/analyze_multi_boundary_capture_20260915.py and write
the outputs to the run's boundary_review folder. Use the map title
"70 Collins multi-boundary survey". Treat the five-second Pause intervals as
the intended segment separators. Do not build an autonomous mission yet.

Present the numbered candidate segments overlaid on one map. Create the
editable segment_labeling_worksheet.csv and summarize each segment's local
time, length, closure gap, area, and RTK Fixed fraction. Preserve all
deliberately driven transitions. Ask me to identify or correct every boundary,
transition, obstacle loop, and excluded segment before proceeding.
```

## 6. Complete the editable segment index

Edit only the human-review columns in
`boundary_review/segment_labeling_worksheet.csv`:

- `label` — plain-language name, such as `rear yard` or `oak tree #1`;
- `role` — `transition`, `guided transition`, `polygon-<name>`, or `obstacle`;
- `include` — `yes`, `no`, or `obstacle`;
- `notes` — closure issues, RTK loss, direction, access restrictions, or other
  facts the builder must preserve.

Do not change the timing, geometry, length, or source-file columns.

### Prompt after editing the index

```text
I updated the editable segment index for the 70 Collins capture. Review the
worksheet, candidate CSVs, summary, and numbered overview map. Check that each
included polygon closes sensibly, each obstacle is a separate usable loop,
and every intended transition remains in the correct drive order. Identify
gaps, overlaps, shortcuts, ambiguous geometry, or low-RTK sections. Ask any
clarifying questions needed before generating mission geometry.
```

Answer the questions explicitly. At minimum decide:

- which segment or segments form each outer boundary;
- the order in which the master mission should visit the polygons;
- which recorded transitions connect them;
- obstacle clearance beyond each recorded obstacle loop;
- whether the manually driven outer boundary is executed or used only as the
  source for inner rings;
- lane spacing, mission speed, lookahead, and planning turn radius;
- rings only or rings plus stripes;
- what to do where a coverage path cannot be safely connected.

## 7. Request the master mission package

For the first 70 Collins mission, use rings only unless a previous mission has
already demonstrated reliable ring execution at that site. Add stripes only
after the ring mission has been reviewed and field-tested.

### Prompt to build the mission

```text
Build the reviewed 70 Collins master mission from the labeled candidate
segments.

Requirements:
- preserve the approved recorded transitions and their original order;
- generate the approved outer-boundary and/or inner-ring coverage for every
  polygon;
- keep stripes disabled for version 1 unless I explicitly approve them;
- use <speed> m/s at every waypoint and <lookahead> m lookahead;
- use <lane spacing> for adjacent coverage passes;
- use the validated left/right turning-radius assumptions and do not silently
  reduce a radius to make geometry fit;
- expand every obstacle loop by its approved additional safety distance;
- never route through an obstacle exclusion or create an unreviewed shortcut;
- if a ring exists geometrically but cannot be safely entered and exited,
  retain the approved manually driven boundary and mark the omitted coverage
  explicitly;
- produce one continuous master mission so no laptop interaction is needed
  between polygons;
- retain the guarded Manual/Pause to AUTO forward-only recovery behavior;
- create a map, waypoint audit CSV, machine-readable report, exact checksum,
  recovery validation, and a supervised blades-off launcher;
- do not enable the web dashboard until I approve the final route image.

Use the 62 Collins scripts only as reviewed reference implementations. Create
70 Collins-specific copies with paths, segment mappings, obstacle indices,
output names, expected row counts, and checksums derived from the new capture.
Run the builders and validations, present the final route map, and list every
remaining limitation or omitted coverage area.
```

Replace every angle-bracket value before sending the prompt. If a value is not
known, ask Codex to stop and request it rather than infer it.

## 8. Critical 62 Collins scripts and how to use them for 70 Collins

The Python files in
`field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260915_master_boundary_replay`
are important, but they are site-specific. Do not run them unchanged against
70 Collins.

### `build_master_boundary_replay_20260915.py`

Purpose:

- reads the reviewed segment worksheet and candidate segment CSVs;
- concatenates the selected paths in recorded order;
- creates an exact-drive review mission and individual segment/composite paths;
- extracts the telephone-pole loop;
- writes GeoJSON, a waypoint audit, report, and review map.

This was the geometry-audit stage, not the final coverage mission. Its 0.5 m/s
exact replay helped verify phase order, connections, and obstacle extraction.

For 70 Collins, create a dated 70-specific version and update:

- `REVIEW`, `OUT`, and output filenames;
- selected segment numbers and any omitted segments;
- composite polygon mappings;
- obstacle segment numbers and closed-loop indices;
- site name, plot title, speed, and report notes.

### `build_rings_only_coverage_20260915.py`

Purpose:

- converts reviewed boundaries into drivable inner rings;
- applies lane spacing, boundary outset, waypoint spacing, speed, lookahead,
  and turn-radius rules;
- selects contained connectors between rings and recorded transitions;
- expands the telephone-pole exclusion;
- retains a safe boundary-only fallback when the over-road inner ring cannot
  be connected;
- writes the final partial master mission, waypoint audit, report, and map.

This script produced the mission currently selected by the web dashboard. It
contains hard-coded 62 Collins field definitions and telephone-pole details.

For 70 Collins, create a new version only after the segment index is approved.
Define the new `FIELDS` sequence from the worksheet, specify all obstacle
loops, and replace every 62-specific path, constant, message, output name, and
fallback. A new site may need more than one obstacle and must not assume that
the 62 Collins pole-bypass logic applies unchanged.

### `validate_master_recovery_20260915.py`

Purpose:

- imports the guarded Pure Pursuit controller;
- simulates Manual travel followed by AUTO reacquisition at every waypoint;
- records acquired and deliberately blocked locations;
- identifies ambiguous crossings or nearby future branches.

The current 62 script points to the earlier exact boundary replay. For
70 Collins, the new validator must point to the **final coverage mission and
its matching audit CSV**, not the intermediate exact replay. Run controller
unit tests as well:

```bash
python3 -m unittest tractor_rpi/testing/test_pure_pursuit_forward_recovery_20260915.py
```

Review all blocked ranges. They are safe refusals, but the operator needs to
know where Manual movement or better alignment may be required.

### Supporting runtime files

- `pure_pursuit_controller_20260915.py` provides RTK/heading gates,
  handheld-mode gating, frozen progress in Manual/Pause, and guarded
  forward-only reacquisition.
- The site launcher verifies the exact mission checksum and expected report,
  runs preflight, checks the start pose, starts logging, and starts the
  controller. It must support `--dashboard`, UDP control `6011`, and telemetry
  `6012`.
- `mission_dashboard_20260910.py` currently selects one mission package with
  `MISSION`, `AUDIT`, `LAUNCHER`, and `EXPECTED_CONFIRMATION` constants. For
  70 Collins, update those constants and the visible title/confirmation text,
  then confirm the API returns equal waypoint and phase counts.

## 9. Review and approve the generated mission

Do not create or enable a launcher from filenames alone. Inspect the map and
report and explicitly confirm:

- polygon and obstacle labels are correct;
- route order matches the intended field order;
- all connectors follow approved recorded paths or reviewed contained paths;
- obstacle buffers use the requested additional clearance;
- every planned ring is inside its allowed drive area;
- omitted rings or polygons are plainly identified;
- stripes are present only if approved;
- every waypoint has the intended speed and lookahead;
- maximum waypoint gap is within the configured limit;
- start location and heading are practical;
- the estimated duration is acceptable;
- the mission checksum is pinned by the launcher.

When the image is approved, request the launcher and dashboard integration:

```text
I reviewed and approve the 70 Collins master mission image and the listed
limitations. Create a supervised blades-off launcher with exact checksum,
report, preflight, RTK/heading, start-pose, duplicate-process, logging, and
operator-confirmation gates. Add dashboard mode using control port 6011 and
telemetry port 6012. Then integrate this mission, its audit CSV, launcher, map
title, limitations, and exact confirmation phrase into the real-time web
dashboard. Run syntax, verification-only, controller-unit, mission/audit
alignment, and local HTTP/API smoke tests. Do not start tractor hardware.
```

## 10. Run through the web dashboard

After the completed package is committed, pushed, and pulled onto the tractor:

```bash
cd /home/al/tractor2025
git pull
python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py
```

Open the printed ZeroTier URL. Before pressing **START MISSION**, confirm the
page title, route map, waypoint count, site limitations, live tractor position,
fresh Teensy state, and handheld Pause state all belong to 70 Collins.

The dashboard START action must call the 70 Collins launcher with
`--dashboard`. The launcher, not the browser, remains responsible for the
checksum, mission contents, preflight, RTK/heading, start pose, logger, speed
cap, and controller arguments.

For the first run:

- blades disengaged;
- handheld within reach;
- start in Pause;
- select AUTO only after the controller is live and waiting;
- use Manual/Pause immediately for unexpected motion;
- after RTK loss, remain Manual until LED4 is green and the tractor is near
  and aligned with the intended forward path;
- preserve and review both the field logger CSV and Pure Pursuit CSV after the
  run.

## Completion checklist

- [ ] Base RTCM service healthy
- [ ] Mission preflight passed
- [ ] Raw survey captured with five-second Pause separators
- [ ] Tractor and development-machine SHA-256 hashes match
- [ ] Candidate segments and overview map created
- [ ] Editable segment worksheet reviewed and completed
- [ ] Every boundary, transition, obstacle, and exclusion decision confirmed
- [ ] Exact replay/audit package reviewed
- [ ] Coverage builder created specifically for 70 Collins
- [ ] Final mission map and limitations explicitly approved
- [ ] Final mission and audit contain the same waypoint count
- [ ] Speed, lookahead, spacing, obstacle, and maximum-gap checks passed
- [ ] Recovery validation points to the final mission
- [ ] Launcher verification-only mode passed
- [ ] Dashboard HTTP/API smoke test passed
- [ ] Changes committed, pushed, and pulled onto the tractor
- [ ] First run performed supervised with blades off
