# Interactive Site Coverage Mission Workflow

This workflow replaces `Site_01_path_planning_tool_v4.ipynb` for the current
non-ROS tractor stack. It starts with decimal-degree latitude/longitude recorded
by `field_test_logger_20260728.py` and ends with the exact five-column mission
format read by `pure_pursuit_controller_20260714.py`:

```text
lat lon yaw_rad lookahead_m speed_mps
```

It is deliberately not a one-button planner. There are four operator
checkpoints:

1. Review the logged boundary and optional obstacle outlines.
2. Compare stripe angles and choose one for local conditions.
3. Review/edit the generated headlands and stripe segments.
4. Independently validate the executable mission.

The tools save plots and plain CSV files at each checkpoint. CSVs can be opened
and edited in Excel, LibreOffice Calc, or a text editor.

## What changed from the ROS 1 notebook

| Old notebook process | Current process |
|---|---|
| ROS bag `/fix` and `/odom` extraction | `field_test_logger` CSV `lat`/`lon` |
| RTCM-base-offset x/y as the source of truth | Decimal-degree lat/lon as the source of truth |
| PlotJuggler timestamps plus workbook tagging | Filtered plot plus editable `include`/`sequence` CSV |
| Many scripts modifying many workbook sheets | Three command-line tools with named artifacts |
| Hard-coded `angle_degrees = 19` | `--angle-degrees 19` or any other angle |
| Hard-coded obstacle coordinates | Optional obstacle CSV fitted from a logged outline |
| Manual sheet copy/paste to combine paths | Reviewed segments plus contained Dubins connectors |
| Mission exported as x/y | Mission exported directly as lat/lon |
| Visual inspection only | Visual inspection plus machine-readable validation report |

The useful ideas from the notebook remain: inward headland paths, adjustable
Boustrophedon stripes, circular obstacle inflation, turn-radius-aware connectors,
curvature-aware review, and plots before every irreversible step. Headland
corners are additionally rounded inward using the configured turn radius.

## Files

Mission-planning tools live in
`tractor_rpi/pure-pursuit/mission_planning/`. Windows collection and
post-run analysis tools live in `field_testing/tools/`, while saved field-test
artifacts live in `field_testing/sites/`.

| File | Purpose |
|---|---|
| `site_boundary_from_log_20260724.py` | Automatically trim/extract a logged boundary (or use explicit rows), import a GeoJSON/KML/KMZ polygon, finalize it, and fit optional circular obstacles |
| `site_coverage_planner_20260724.py` | Compare angles, preview coverage, and build a candidate mission |
| `validate_site_mission_20260724.py` | Independently audit and plot the five-column mission |
| `archive_site_mission_20260724.py` | Verify PASS and archive the reviewed deployment/audit package into Git |
| `field_testing/tools/field_test_analysis_menu_20260726.ps1` | Menu for collecting RTK-base data, collecting tractor run data, and generating the final analysis/map |
| `field_testing/tools/collect_site_run_20260724.ps1` | Download and remotely verify completed-run logs plus the tractor mission-package snapshot |
| `field_testing/tools/collect_rtkbase_esp32_20260724.ps1` | Trigger the RTK-base ESP32 download and collect its verified CSV on Windows |
| `field_testing/tools/analyze_run_20260726.py` | Calculate time-weighted CTE results and generate the standalone interactive HTML map |
| `site_planner_common_20260724.py` | Shared projection, CSV, curvature, and Dubins helpers |
| `requirements_site_planner_20260724.txt` | Shapely and Matplotlib dependencies |

The local east/north projection exactly matches the current controller:

```text
x_east  = 111320 × (lon - ref_lon) × cos(ref_lat)
y_north = 110540 × (lat - ref_lat)
```

This is appropriate for a mower-sized site. Lat/lon remains authoritative in
reviewed files and in the final mission.

## One-time setup on RPi5NAS

```bash
cd ~/repos/tractor2025/tractor_rpi/pure-pursuit/mission_planning
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements_site_planner_20260724.txt
```

Reactivate this environment in a new terminal with:

```bash
cd ~/repos/tractor2025/tractor_rpi/pure-pursuit/mission_planning
source .venv/bin/activate
```

Create a separate working directory for each site. Generated site files should
normally remain outside the source-code directory:

```bash
mkdir -p ~/field_plans/site_01
```

## Optional one-time setup on Windows 10

Use PowerShell from the Windows 10 laptop. The examples assume the repository
is at `C:\Repos\tractor2025`, which is the current location on this laptop.
First check that Python 3.11 or newer is available:

```powershell
python --version
```

If that command is missing or only opens the Microsoft Store, install the
[official Python Install Manager for Windows](https://www.python.org/downloads/)
and follow its prompt to install a current Python runtime. The
[official Windows Python documentation](https://docs.python.org/3/using/windows.html)
also covers command and app-execution-alias troubleshooting.

```powershell
cd C:\Repos\tractor2025\tractor_rpi\pure-pursuit\mission_planning
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements_site_planner_20260724.txt
```

If PowerShell refuses to run `Activate.ps1`, allow local script execution only
for the current PowerShell window, then activate again:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

Reactivate the environment in each new PowerShell window and define convenient
locations for this session:

```powershell
$repo = 'C:\Repos\tractor2025'
$planner = Join-Path $repo 'tractor_rpi\pure-pursuit\mission_planning'
$site = Join-Path $env:USERPROFILE 'Documents\field_plans\site_01'

Set-Location $planner
.\.venv\Scripts\Activate.ps1
New-Item -ItemType Directory -Force $site
```

The Linux commands later in this guide remain the primary RPi5NAS examples.
On Windows:

| RPi5NAS form | Windows PowerShell form |
|---|---|
| `python3` | `python` after activating `.venv` |
| `source .venv/bin/activate` | `.\.venv\Scripts\Activate.ps1` |
| `~/field_plans/site_01/file.csv` | `"$site\file.csv"` |
| trailing `\` for a continued command | trailing backtick `` ` `` |

Keep site artifacts outside the Git repository unless there is a deliberate
reason to version a reviewed boundary or mission.

## Step 1 — Capture and review the site outline

### 1A. Log one clean perimeter lap

The current field logger has GPS-only operation, so the Teensy does not need to
be connected for a walking survey or other controlled capture. Its dedicated
GPS logging feed is UDP 6009.

Run the logger on the tractor RPi:

```bash
cd ~/tractor2025/tractor_rpi
python3 field_test_logger_20260728.py \
  --output ~/field_plans/site_01/00_boundary_log.csv
```

Recommended capture practice:

- Wait for `RTK Fixed` before starting the useful lap.
- At the intended start/end point, remain stopped for approximately 10 seconds.
  Drive the perimeter once, return to the same point, and stop there for
  approximately 10 seconds again. `auto-extract` uses these matching pauses.
- Drive or walk one ordered, closed perimeter without doubling back.
- A second lap is useful evidence, but select one lap for the polygon rather
  than mixing both laps into one sequence.
- Keep the GPS reference point’s location in mind. The planner clearance is a
  tractor-reference-point clearance, not automatically a mower-deck-edge
  clearance.
- Stop the logger with `Ctrl+C` so the CSV is closed cleanly.

#### Copy the completed tractor log to Windows 10

The logger runs on the tractor, so its CSV is not automatically present on the
Windows laptop. Open Windows PowerShell, define the local working folders, and
copy the file over ZeroTier:

```powershell
$repo = 'C:\Repos\tractor2025'
$planner = Join-Path $repo 'tractor_rpi\pure-pursuit\mission_planning'
$site = Join-Path $env:USERPROFILE 'Documents\field_plans\site_01'
New-Item -ItemType Directory -Force $site
scp al@192.168.193.76:/home/al/field_plans/site_01/00_boundary_log.csv "$site\00_boundary_log.csv"
```

Use the actual site directory name on both machines. If ZeroTier is unavailable
but the tractor is on the same LAN, substitute its current LAN address for
`192.168.193.76`.

Verify that the Windows copy has the same line count as the tractor copy:

```powershell
(Get-Content "$site\00_boundary_log.csv" | Measure-Object -Line).Lines
ssh al@192.168.193.76 "wc -l /home/al/field_plans/site_01/00_boundary_log.csv"
```

Reactivate the Windows planner environment in every new PowerShell window:

```powershell
Set-Location $planner
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

If automatic pause matching is unavailable or inappropriate, `extract` still
accepts explicit `--start-row` and `--end-row` values. These are CSV data-row
numbers after the header; the corresponding Excel row is one greater. The
candidate output records the original Excel/CSV source row.

### Optional alternative to 1A and 1B: draw one polygon on a map

Use this route when a field-logged perimeter is unavailable or when a map
polygon is a useful first draft. It avoids copying individual latitude and
longitude values from Google Maps. The boundary tool accepts one polygon from
`.geojson`, `.json`, `.kml`, or `.kmz` and converts it into the same editable
candidate CSV used by the logged-data route.

Public options:

1. **Google My Maps — easiest with familiar aerial imagery.** Open
   [Google My Maps](https://www.google.com/maps/about/mymaps/), create a map,
   choose **Draw a line > Add line or shape**, click the perimeter vertices,
   and close the shape. Google documents the drawing process in
   [Draw lines & shapes in My Maps](https://support.google.com/mymaps/answer/3433053).
   From the map's three-dot menu, export the polygon layer or whole map as
   KML/KMZ.
2. **geojson.io — fastest direct GeoJSON route.** Open
   [geojson.io](https://geojson.io/), zoom to the site, use the polygon tool,
   inspect the vertices, and save the result as GeoJSON. It is a convenient
   public web editor, but imagery and service availability can change.
3. **QGIS — best repeatable desktop option.** QGIS is free and open source,
   runs on Windows, and provides more capable digitizing and layer inspection.
   Use the current Windows installer from the
   [official QGIS download page](https://www.qgis.org/download/), create a
   polygon layer in `EPSG:4326 - WGS 84`, digitize one polygon, and export that
   layer as GeoJSON or KML. The QGIS Long Term Release is the conservative
   choice if this becomes a regular workflow.

For today's Windows 10 workflow, Google My Maps plus KML/KMZ export is the
simplest starting point. The input file may remain in `Downloads`; the generated
CSV and PNG should go in the site's working directory. For example:

```powershell
Set-Location $planner
New-Item -ItemType Directory -Force $site

python site_boundary_from_log_20260724.py import-map `
  "$env:USERPROFILE\Downloads\site_outline.kmz" `
  --output "$site\01_boundary_candidates.csv" `
  --plot "$site\01_boundary_candidates.png"

Invoke-Item "$site\01_boundary_candidates.png"
Invoke-Item "$site\01_boundary_candidates.csv"
```

Use the actual downloaded extension and filename; `.kmz` and `.geojson` work
the same way. The RPi5NAS equivalent is:

```bash
python3 site_boundary_from_log_20260724.py import-map \
  ~/field_plans/site_01/site_outline.kml \
  --output ~/field_plans/site_01/01_boundary_candidates.csv \
  --plot ~/field_plans/site_01/01_boundary_candidates.png
```

The import deliberately rejects files containing multiple polygons or mixed
geometry. Export only the intended site boundary layer. Then review:

- `01_boundary_candidates.png` for shape, scale, vertex order, and start point;
- `01_boundary_candidates.csv` in Excel for `include`, `sequence`, and notes;
- the polygon against known physical features, property limits, obstacles,
  slopes, ditches, and the room needed for headland turns.

If the PNG and CSV are correct, **skip Step 1B**. Map import has already created
the candidate boundary that Step 1B would have extracted from a field log.
Continue directly with **Step 1C — Finalize and validate the polygon**.

Satellite/aerial imagery is not an RTK survey. Rooflines, trees, shadows,
seasonal imagery, and tile alignment can move the apparent edge by enough to
matter to a mower. Treat a map polygon as a draft, use a conservative
`boundary-clearance-m`, and verify critical edges in the field. A strong hybrid
workflow is to draw the coarse polygon on the laptop, then replace or adjust
critical vertices with RTK-logged evidence.

### 1B. Automatically trim the lap and extract a candidate boundary

The preferred route for a manually driven perimeter is `auto-extract`. It:

1. finds contiguous pauses below `0.10 m/s` lasting at least 8 seconds;
2. favors pause pairs close to the intended 10-second duration;
3. requires both pause centroids to be within 1.0 m;
4. uses the last fully settled sample (`≤0.05 m/s`) before departure and the
   first fully settled sample after return as the extraction limits; and
5. immediately feeds those limits into the normal RTK filtering and
   spatial-thinning extractor.

Windows 10 PowerShell (one line to avoid accidental trailing backticks):

```powershell
python .\site_boundary_from_log_20260724.py auto-extract "$site\00_boundary_log.csv" --output "$site\01_boundary_candidates.csv" --plot "$site\01_boundary_candidates.png" --stationary-speed-mps 0.10 --pause-edge-speed-mps 0.05 --pause-seconds 8 --target-pause-seconds 10 --same-place-radius-m 1.0 --minimum-lap-seconds 30 --min-spacing-m 0.50 --fix-quality "RTK Fixed"
```

The command prints every qualifying pause, the selected pair, extractor
data-row limits, and corresponding Excel rows. Always inspect the generated PNG
before accepting the automatic choice.

RPi5NAS:

```bash
cd ~/repos/tractor2025/tractor_rpi/pure-pursuit/mission_planning
source .venv/bin/activate

python3 site_boundary_from_log_20260724.py auto-extract \
  ~/field_plans/site_01/00_boundary_log.csv \
  --output ~/field_plans/site_01/01_boundary_candidates.csv \
  --plot ~/field_plans/site_01/01_boundary_candidates.png \
  --stationary-speed-mps 0.10 \
  --pause-edge-speed-mps 0.05 \
  --pause-seconds 8 \
  --target-pause-seconds 10 \
  --same-place-radius-m 1.0 \
  --minimum-lap-seconds 30 \
  --min-spacing-m 0.50 \
  --fix-quality "RTK Fixed"
```

Manual fallback for a known row range:

```powershell
$startRow = 766
$endRow = 2264
python .\site_boundary_from_log_20260724.py extract "$site\00_boundary_log.csv" --output "$site\01_boundary_candidates.csv" --plot "$site\01_boundary_candidates.png" --start-row $startRow --end-row $endRow --min-spacing-m 0.50 --fix-quality "RTK Fixed"
```

The values above are the worked Collins Drive example; replace them with the
detected or reviewed data-row limits for another capture.

Optional extraction filters:

- `--end-row N` limits the end of the chosen lap.
- `--max-hdop 1.0` rejects blank or worse HDOP values.
- `--fix-quality ""` accepts any fix quality. Do this only for diagnostics, not
  a production boundary.

Open both outputs:

- `01_boundary_candidates.png` shows all accepted fixes in gray and the
  spatially thinned candidates in blue.
- `01_boundary_candidates.csv` is the manual checkpoint.

In the CSV:

- Set `include` to `y` or `n`.
- Edit `sequence` to define one non-crossing perimeter order.
- Use `notes` to identify gates, trees, wet ground, property corners, or other
  evidence.
- The first included sequence becomes the default preferred headland splice
  anchor. The builder may move the actual mission start along the headland to
  find a contained forward-only transition.

### 1C. Finalize and validate the polygon

Windows 10 PowerShell:

```powershell
python .\site_boundary_from_log_20260724.py finalize "$site\01_boundary_candidates.csv" --output "$site\01_boundary_final.csv" --plot "$site\01_boundary_final.png" --simplify-m 0.10
Invoke-Item "$site\01_boundary_final.png"
Invoke-Item "$site\01_boundary_final.csv"
```

RPi5NAS:

```bash
python3 site_boundary_from_log_20260724.py finalize \
  ~/field_plans/site_01/01_boundary_candidates.csv \
  --output ~/field_plans/site_01/01_boundary_final.csv \
  --plot ~/field_plans/site_01/01_boundary_final.png \
  --simplify-m 0.10
```

The command refuses self-intersections and suspiciously small areas. A 0.10 m
simplification removes small RTK wiggles while preserving topology. Use
`--simplify-m 0` to retain every included point. Use `--reverse` if the selected
perimeter direction is wrong.

Do not continue until the filled polygon and numbered start point look correct.

### Optional: fit a circular obstacle

Capture a separate ordered outline around a tree, pole, garden, or other
approximately circular exclusion. Run `extract`, review its candidate CSV, then:

```bash
python3 site_boundary_from_log_20260724.py fit-obstacle \
  ~/field_plans/site_01/01_tree_candidates.csv \
  --name "Tree 1" \
  --output ~/field_plans/site_01/01_obstacles.csv \
  --plot ~/field_plans/site_01/01_tree_circle.png \
  --safety-margin-m 0.75
```

Unlike the old least-squares mean-radius result, the saved radius is expanded to
contain every included outline fix. `safety_margin_m` is then added again by
the planner. The plot shows all three circles.

For an irregular obstacle, use a deliberately conservative circle. If that is
too wasteful, a future polygon-obstacle extension is preferable to shrinking
the circle unsafely.

## Step 2 — Choose the Boustrophedon angle

Screen angles before generating the reviewed stripe table:

Windows 10 PowerShell, with the requested 39-inch (`0.9906 m`) row spacing and
no obstacle file:

```powershell
python site_coverage_planner_20260724.py compare-angles "$site\01_boundary_final.csv" --output-dir "$site" --angles 0:175:5 --lane-spacing-m 0.9906 --stripe-end-trim-m 3.0 --boundary-clearance-m 0.75 --headland-passes 2

Invoke-Item "$site\01_angle_comparison.png"
Invoke-Item "$site\01_angle_comparison.csv"
```

RPi5NAS, with an optional obstacle file:

```bash
python3 site_coverage_planner_20260724.py compare-angles \
  ~/field_plans/site_01/01_boundary_final.csv \
  --obstacles ~/field_plans/site_01/01_obstacles.csv \
  --output-dir ~/field_plans/site_01 \
  --angles 0:175:5 \
  --lane-spacing-m 0.9906 \
  --stripe-end-trim-m 3.0 \
  --boundary-clearance-m 0.75 \
  --headland-passes 2
```

Outputs:

- `01_angle_comparison.csv`
- `01_angle_comparison.png`

The PNG shows six actual coverage-line layouts rather than a multi-axis
engineering graph. The choices are centered around the orientation producing
the fewest rows, plus nearby rotations and an approximately perpendicular
comparison. Each panel translates the math-frame angle into the two compass
headings, and reports row count, estimated end-turn count, and average row
length. Blue arrows are included rows; red dotted pieces are too short and
would be excluded.

The CSV retains results for every evaluated angle. It includes compass
bearings, row and end-turn counts, short-segment count, average row length, and
the original screening-distance columns for deeper inspection.

The fewest-row result is not automatically the best operating choice. Actual
keyhole-turn geometry is not drawn or validated until the later
preview/build work. Also consider:

- slope direction and rollover risk;
- drainage, wet ground, and rutting;
- mower discharge direction;
- sun/glare or camera visibility;
- where the operator wants the tractor to start and finish;
- the open space needed for turns.

### Angle convention

`--angle-degrees` is a local math-frame angle:

- `0°` = east/west stripe axis;
- `90°` = north/south stripe axis;
- positive angles rotate counterclockwise from east;
- values repeat every 180° because a stripe is an undirected axis.

The notebook’s important value remains a normal input:

```text
--angle-degrees 19
```

A 19° math-frame forward heading is a 71° compass heading; the opposite stripe
direction is 251° compass.

## Step 3 — Preview and edit the coverage plan

Carry the selected Step 2 angle into `preview` through `--angle-degrees`. For
the Collins Drive site, the selected value is `105`:

```powershell
$angle = 105

python site_coverage_planner_20260724.py preview "$site\01_boundary_final.csv" --output-dir "$site" --angle-degrees $angle --lane-spacing-m 0.9906 --stripe-end-trim-m 3.0 --boundary-clearance-m 0.75 --headland-passes 2 --turn-radius-m 1.90 --stripe-turn-policy keyhole --waypoint-spacing-m 0.50 --straight-speed-mps 0.75 --outer-headland-speed-mps 0.50 --headland-speed-mps 0.75 --turn-speed-mps 0.50 --scan-from low --first-stripe-direction reverse --ring-direction clockwise

Invoke-Item "$site\02_coverage_preview.png"
Invoke-Item "$site\02_coverage_segments.csv"
```

Before proceeding to Step 4, verify that the newly written settings contain
the selected radius and turn policy:

```powershell
Get-Content "$site\02_plan_settings.json" -Raw | ConvertFrom-Json | Select-Object angle_degrees, lane_spacing_m, turn_radius_m, stripe_turn_policy, straight_speed_mps, outer_headland_speed_mps, headland_speed_mps, turn_speed_mps
```

For Collins Drive, the expected values are `105`, `0.9906`, `1.9`, and
`keyhole`. If `turn_radius_m` still reports `3.0` or the policy is blank, Step 3
was not rerun with the current command; do not start Step 4.

RPi5NAS, with an optional obstacle file:

```bash
python3 site_coverage_planner_20260724.py preview \
  ~/field_plans/site_01/01_boundary_final.csv \
  --obstacles ~/field_plans/site_01/01_obstacles.csv \
  --output-dir ~/field_plans/site_01 \
  --angle-degrees 105 \
  --lane-spacing-m 0.9906 \
  --stripe-end-trim-m 3.0 \
  --boundary-clearance-m 0.75 \
  --headland-passes 2 \
  --turn-radius-m 1.90 \
  --stripe-turn-policy keyhole \
  --waypoint-spacing-m 0.50 \
  --scan-from low \
  --first-stripe-direction reverse \
  --ring-direction clockwise
```

For `105°`, `reverse` makes the first stripe heading approximately `165°`
compass (south-southeast). With `scan-from low`, the following stripe is on the
tractor's right and is driven at approximately `345°` compass
(north-northwest). This matches the requested alternating-row setup before the
keyhole turn geometry itself is generated and reviewed.

Outputs:

- `02_plan_settings.json` — records every geometric and controller parameter.
- `02_coverage_segments.csv` — editable stripe checkpoint.
- `02_coverage_preview.png` — boundary, safe area, headlands, obstacles,
  numbered stripe arrows, and the preferred splice anchor.

### Parameters that must be measured/reviewed

| Parameter | Meaning |
|---|---|
| `lane-spacing-m` | Center-to-center cut spacing. The selected 39-inch spacing is `0.9906 m`, giving about 3 inches of nominal overlap with the 42-inch deck. Verify actual cut width and GPS tracking. |
| `boundary-clearance-m` | Distance from the logged perimeter to the tractor reference point. Set it from deck/body overhang, survey meaning, GPS error, and desired safety margin. |
| `headland-passes` | Perimeter coverage passes before interior stripes. This creates working room but does not by itself guarantee a feasible U-turn. |
| `outer-headland-follows-boundary` | Opt-in only when the finalized, manually driven boundary already includes the required property/homeowner safety buffer. Pass 1 follows that boundary with tractor-radius corner rounding and no uniform inset. Later perimeter passes retain their normal inset geometry. |
| `stripe-end-trim-m` | Distance removed from both ends of every clipped stripe to leave turning room. This replaces the notebook’s separate “shorten stripes” script and is visible in the editable CSV/plot. |
| `turn-radius-m` | Common minimum planning radius for left and right turns. Manual circle tests measured approximately 1.05 m full-left and 1.63 m full-right. This site uses 1.90 m: 0.27 m (about 17%) gentler than the weaker measured right turn. Do not plan at the measured limit without repeatability tests. |
| `stripe-turn-policy` | `keyhole` constrains adjacent-row transitions to compact three-arc agricultural omega turns. `shortest-dubins` is retained for engineering comparison and must not be used merely to force a build. |
| `outer-headland-speed-mps` | Optional speed for perimeter pass 1. If omitted, pass 1 uses `headland-speed-mps` like the other perimeter passes. |
| `headland-speed-mps` | Speed for perimeter pass 2 and any later perimeter passes. |
| `waypoint-spacing-m` | Maximum sampling gap. 0.50 m is denser than the current controller’s historical 1 m examples. |
| lookahead/speed options | Straight, turn, and headland values are saved separately. Start conservatively. |

Open `02_coverage_segments.csv` in Excel:

- `include`: `y`/`n`; short pieces are visible but initially set to `n`.
- `sequence`: execution order.
- `reverse`: set to `y` to swap that segment’s endpoints.
- `start_lat/lon`, `end_lat/lon`: authoritative endpoints. They may be trimmed
  manually, but every edit must remain inside the plotted safe area.
- local east/north columns are inspection aids and are recalculated from
  lat/lon during build.

Do not edit `02_plan_settings.json` casually. Rerunning `preview` with explicit
arguments gives an auditable replacement.

Useful orientation controls:

- `--scan-from low|high` changes which side is cut first.
- `--first-stripe-direction forward|reverse` selects angle vs. angle+180° for
  the first stripe.
- `--start-lat` and `--start-lon` move the preferred headland splice anchor.
  The builder searches nearby ring splice points and the final mission plot
  shows the actual selected start.
- `--ring-direction clockwise|counterclockwise` changes headland direction.

## Step 4 — Build, validate, and only then field-test

### Required keyhole-turn policy checkpoint

With `stripe-turn-policy` set to `keyhole`, adjacent rows use compact
three-arc omega turns. Because the 39-inch row spacing is less than twice the
tractor's turn radius, a forward-only left-then-right two-arc path cannot finish
aligned with the adjacent row. The third, smaller correcting arc is
kinematically necessary.

When the next row is on the right, the normal pattern is `LRL`
(left-right-left); when it is on the left, the mirror is `RLR`. The first
Collins Drive transition is next to the polygon edge and cannot turn away from
the row while remaining contained, so it uses a clearly recorded
`boundary-fallback` omega in the opposite orientation. All later tested row
transitions use the preferred turn-away-first pattern. Review every orange
connector in the mission preview before validation or field use. A red triangle
marks each boundary-constrained fallback requiring special attention.

### 4A. Build the candidate executable mission

Windows 10 PowerShell:

```powershell
$mission = Join-Path $site '62_Collins_Dr_mission.txt'

python site_coverage_planner_20260724.py build --settings "$site\02_plan_settings.json" --segments "$site\02_coverage_segments.csv" --output $mission --strict-curvature

if ($LASTEXITCODE -eq 0) {
  Invoke-Item "$site\62_Collins_Dr_mission_preview.png"
  Invoke-Item "$site\62_Collins_Dr_mission_audit.csv"
  Get-Content "$site\62_Collins_Dr_mission_build_report.json" -Raw
} else {
  Write-Warning "Build failed; no mission artifacts were written."
}
```

RPi5NAS:

```bash
python3 site_coverage_planner_20260724.py build \
  --settings ~/field_plans/site_01/02_plan_settings.json \
  --segments ~/field_plans/site_01/02_coverage_segments.csv \
  --output ~/field_plans/site_01/site_01_mission.txt
```

The builder:

1. Reconstructs the reviewed safe polygon.
2. Creates/densifies headland and stripe path pieces.
3. Uses the selected compact keyhole/omega policy for adjacent coverage rows.
4. Uses contained Dubins-family searches for headland-to-headland and
   headland-to-first-stripe transitions.
5. Refuses to write a mission if a connector or manual edit leaves that
   polygon.
6. Converts every local point back to lat/lon and writes the controller’s exact
   five-column format.

Additional outputs are:

- `site_01_mission_audit.csv`
- `site_01_mission_build_report.json`
- `site_01_mission_preview.png`

If `build` prints `ERROR`, it deliberately does not write the mission, preview,
audit, or build report. Do not run `Invoke-Item` for those outputs after a
failed build; correct the reported geometry problem and rerun `build`.

If a connector fails, this is useful information—not a reason to bypass the
check. Try, in this order:

1. Reverse the failing stripe in the editable CSV.
2. Trim its endpoint farther from a tight corner.
3. Change stripe sequence or the selected starting side.
4. Increase headland room or choose a different stripe angle.
5. Use a separate mission for a disconnected/narrow subarea.
6. Reduce the turn radius only after measured tractor geometry supports it.

`--strict-curvature` also refuses sampled offset-polygon corners below 90% of
the configured radius. Without it, those corners produce a prominent warning
because Pure Pursuit lookahead may smooth them, but static polygon geometry
alone cannot prove the driven radius.

### 4B. Independently validate

Windows 10 PowerShell:

```powershell
python validate_site_mission_20260724.py $mission --settings "$site\02_plan_settings.json"

Invoke-Item "$site\62_Collins_Dr_mission_validation.png"
Get-Content "$site\62_Collins_Dr_mission_validation.json" -Raw
```

Optional hard curvature gate:

```powershell
python validate_site_mission_20260724.py $mission --settings "$site\02_plan_settings.json" --strict-curvature
```

RPi5NAS:

```bash
python3 validate_site_mission_20260724.py \
  ~/field_plans/site_01/site_01_mission.txt \
  --settings ~/field_plans/site_01/02_plan_settings.json
```

Outputs:

- `site_01_mission_validation.json`
- `site_01_mission_validation.png`

Checks include:

- exactly five numeric values per mission row;
- positive lookahead and nonnegative speed;
- speed cap;
- duplicate points and maximum waypoint gap;
- stored yaw versus the next path segment;
- sampled local curvature;
- full path containment in the reconstructed boundary/obstacle safe area.

Exit status and report status:

- `PASS`: static checks passed.
- `REVIEW`: no hard failure, but warnings require visual/engineering review.
- `FAIL`: do not load the mission into the live controller.

Use `--strict-curvature` when offset-polygon corners have been smoothed or
otherwise proven feasible and a hard minimum-radius gate is desired.

### 4C. Version the validated mission in Git

The per-site working directory remains outside the repository while planning.
After static validation passes and the visual checkpoints are accepted, run
the archive helper from the activated mission-planning environment:

```powershell
python archive_site_mission_20260724.py "$site" --launcher-max-speed-mps 0.75
```

The site directory name becomes both the Git directory name and the default
mission prefix. For example, a working directory named `62_Collins_Dr` expects
`62_Collins_Dr_mission.txt` and creates:

```text
tractor_rpi/pure-pursuit/missions/62_Collins_Dr/
```

The helper:

1. Refuses to archive anything without validation status `PASS`.
2. Confirms the validation report names the same mission file.
3. Copies the executable mission, final boundary, reviewed segments, settings,
   audit, build/validation reports, and final PNG checkpoints.
4. Verifies every copied file by SHA-256.
5. Generates a GitHub-renderable site `README.md` with parameters, status,
   mission hash, visual checkpoints, and field hold points.
6. Generates `run_<site-name>_mission.sh`. The launcher resolves the mission,
   controller, and logger paths relative to its own location, so it can be
   started from any directory on the tractor RPi. Before moving the tractor it
   requires validation status `PASS` and verifies the mission's LF-normalized
   SHA-256, so Windows CRLF and Linux LF checkouts verify identically. It then
   starts the field logger in the background and the controller in the
   foreground with the explicitly selected launcher speed cap. For this
   tractor, use `--launcher-max-speed-mps 0.75`; the mission itself retains
   `0.50 m/s` headland/turn speeds and `0.75 m/s` straight speeds.
7. Refuses to overwrite an existing archive accidentally.

If an already archived mission is deliberately superseded by a newly reviewed
and validated version, inspect both versions first, then use:

```powershell
python archive_site_mission_20260724.py "$site" --replace --launcher-max-speed-mps 0.75
```

Review what Git will receive:

```powershell
$siteName = Split-Path $site -Leaf
$archive = Join-Path $repo "tractor_rpi\pure-pursuit\missions\$siteName"

Invoke-Item "$archive\README.md"
git status --short
```

The generated launcher is part of the archived mission package. After the
package has been committed, pushed, and pulled onto the tractor RPi, run it
with `bash` (the executable file bit is not required):

```bash
cd ~/tractor2025/tractor_rpi/pure-pursuit/missions/62_Collins_Dr
bash ./run_62_Collins_Dr_mission.sh
```

The launcher stops the background logger when the controller exits or when
`Ctrl+C` is pressed. Keep the tractor in Manual or Pause until RTK Fixed,
heading, e-stop, and route-start checks are complete. The `0.75 m/s` launcher
cap permits the mission's requested speeds; it does not make every waypoint
run at `0.75 m/s`.

The JSON files should be committed. The settings make the mission reproducible;
the build report records connector decisions; and the validation report records
the exact checks that passed. Do not version `.venv`, temporary angle sweeps,
failed builds, or every intermediate working file.

Mission files expose exact geographic coordinates. Confirm that publishing
those coordinates is intentional before pushing to a public repository.

### 4D. Controller dry run and first field run

The Windows laptop stops after creating and reviewing a `PASS`/acceptable
`REVIEW` validation report. Do not try to run the live tractor controller from
Windows. Transfer the reviewed mission to the onboard Raspberry Pi, then use
the RPi command below only after the keyhole-turn preview and all static checks
are satisfactory.

The final file has no header and is directly read by:

```bash
cd ~/repos/tractor2025/tractor_rpi/pure-pursuit
python3 pure_pursuit_controller_20260714.py \
  ~/field_plans/site_01/site_01_mission.txt \
  --mode live \
  --max-speed 0.30
```

Before powered autonomous motion:

1. Confirm the plotted start is physically accessible and position the tractor
   aligned with the first path leg.
2. Verify e-stop, Pause/Manual/Auto mode behavior, steering sign, and neutral.
3. Confirm live `RTK Fixed` and valid dual-antenna heading.
4. First observe a no-motion/dry-run command stream.
5. Use a supervised low-speed run with a person at the e-stop.
6. Review the pursuit log against the mission before raising speed.

Static validation is not evidence that terrain is traversable or that the mower
deck, trees, people, animals, drop-offs, or moving objects are clear.

### 4E. Collect the completed run on Windows

After the tractor is parked, its engine is off, and both programs have closed
their logs, note the common timestamp printed in the two filenames. From the
repository root on Windows, start the menu:

```powershell
.\field_testing\tools\field_test_analysis_menu_20260726.ps1
```

Choose the site by number from the complete mission-package folders under
`tractor_rpi/pure-pursuit/missions/`, then choose an existing downloaded run
from the numbered list. Choose `N` and enter the common run timestamp only when
collecting a run that does not yet have a local run directory. Then choose:

1. Download RTK-base/ESP32 data.
2. Download the tractor logs and mission package.
3. Analyze the run and generate/open the HTML map.

Option 2 defaults to tractor01's ZeroTier address `192.168.193.76`. It checks
SSH connectivity and verifies remote SHA-256 values for:

- `/home/al/repos/field-testing-data/pursuit_log_<run>.csv`;
- `/home/al/field_logs/field_test_<run>.csv`; and
- every file in the site mission directory present on tractor01 at collection
  time.

The destination is:

```text
%USERPROFILE%\Documents\field_plans\<site>\runs\<run>\
```

Existing files are reused only when their hashes match; mismatches stop without
overwriting. The collector records CSV row counts, remote/local hashes,
`file_hashes.csv`, and `collection_summary.json`. Collect immediately after
the run so the copied mission directory represents the package used in the
field.

Option 1 collects the corresponding ESP32 power/environment log through the
RTK base at ZeroTier address `192.168.193.88`. When a saved base file is not
specified, it asks for confirmation before remotely running:

```text
python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete <run-specific-file>
```

The base downloader first saves the CSV under `/home/al/esp32_data/` and only
then clears/reinitializes the ESP32 source file. The PowerShell script copies
that exact base file to the matching Windows run directory and requires its
SHA-256 to match. It retains the base-station copy as a recovery backup by
default.

Option 3 requires Python 3, NumPy, and pandas. One-time setup:

```powershell
python -m pip install -r .\field_testing\tools\requirements_field_testing_analysis_20260726.txt
```

The analyzer reads the collected logs and mission package, calculates
time-weighted controller and geometric path error, and writes these files
directly into the selected run directory:

```text
<site>_run_analysis_<run>.json
<site>_run_map_data_<run>.json
<site>_run_map_<run>.html
```

The HTML is assembled by Python as a complete UTF-8 document. Node.js and
manual template-replacement commands are not required.

The underlying tools can still be run directly when advanced host or path
overrides are needed:

```powershell
.\field_testing\tools\collect_site_run_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr

.\field_testing\tools\collect_rtkbase_esp32_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr

python .\field_testing\tools\analyze_run_20260726.py `
  --site-name 62_Collins_Dr `
  --run-id 20260724_172543 `
  --open-map
```

## Known limitations and recommended next improvements

1. Obstacles are circular. Add reviewed obstacle polygons/GeoJSON for buildings,
   beds, and irregular hazards.
2. A narrow neck that divides the safe polygon is intentionally rejected.
   Proper multi-cell Boustrophedon sequencing should be added before automating
   such a site.
3. Angle comparison does not include terrain elevation or slope. A surveyed
   terrain layer would make the comparison much more meaningful.
4. Headland corners are rounded with the configured radius, but complex
   concave boundaries can still produce tight sampled transitions. The
   independent curvature warning and slow first run remain required.
5. Add GeoJSON/KML export and an aerial-image overlay to the generated
   checkpoint plots. Map polygon import is supported, but the current PNG
   checkpoint intentionally shows geometry without a live basemap.
6. Add mission filename, SHA-256, planner settings, and boundary/obstacle hashes
   to the pursuit log so every test can be reproduced exactly.
7. Add operator event markers to `field_test_logger` to record “boundary start,”
   “boundary end,” and obstacle names while capturing instead of finding row
   ranges afterward.

## Why the workflow can stop

A stopped build is preferable to a plausible-looking unsafe file. The tools
intentionally stop for:

- invalid or self-crossing boundary order;
- no interior remaining after clearance/headlands;
- disconnected safe drive area;
- no included stripe segments;
- a manual segment edit outside the safe polygon;
- a turn-radius-constrained connector that cannot stay inside the polygon;
- final containment failure;
- strict curvature failure when requested.

At each stop, return to the immediately preceding CSV/plot checkpoint, make one
understood change, and regenerate the downstream artifacts.
