# Interactive Site Coverage Mission Workflow

This workflow replaces `Site_01_path_planning_tool_v4.ipynb` for the current
non-ROS tractor stack. It starts with decimal-degree latitude/longitude recorded
by `field_test_logger_20260717.py` and ends with the exact five-column mission
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

Run these from `tractor_rpi/pure-pursuit/mission_planning/`:

| File | Purpose |
|---|---|
| `site_boundary_from_log_20260724.py` | Extract a logged boundary or import a GeoJSON/KML/KMZ polygon, finalize it, and fit optional circular obstacles |
| `site_coverage_planner_20260724.py` | Compare angles, preview coverage, and build a candidate mission |
| `validate_site_mission_20260724.py` | Independently audit and plot the five-column mission |
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

```bash
cd ~/repos/tractor2025/tractor_rpi
python3 field_test_logger_20260717.py \
  --output ~/field_plans/site_01/00_boundary_log.csv
```

The equivalent Windows 10 command is:

```powershell
Set-Location "$repo\tractor_rpi"
python field_test_logger_20260717.py `
  --output "$site\00_boundary_log.csv"
```

Recommended capture practice:

- Wait for `RTK Fixed` before starting the useful lap.
- Drive or walk one ordered, closed perimeter without doubling back.
- A second lap is useful evidence, but select one lap for the polygon rather
  than mixing both laps into one sequence.
- Keep the GPS reference point’s location in mind. The planner clearance is a
  tractor-reference-point clearance, not automatically a mower-deck-edge
  clearance.
- Stop the logger with `Ctrl+C` so the CSV is closed cleanly.

If the CSV contains setup time, multiple laps, or the drive back to the trailer,
open it in Excel first and note the useful data-row range. `--start-row` and
`--end-row` count data rows after the header. The candidate output also records
the actual source CSV row.

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

### 1B. Extract a candidate boundary

```bash
cd ~/repos/tractor2025/tractor_rpi/pure-pursuit/mission_planning
source .venv/bin/activate

python3 site_boundary_from_log_20260724.py extract \
  ~/field_plans/site_01/00_boundary_log.csv \
  --output ~/field_plans/site_01/01_boundary_candidates.csv \
  --plot ~/field_plans/site_01/01_boundary_candidates.png \
  --start-row 1 \
  --min-spacing-m 0.50 \
  --fix-quality "RTK Fixed"
```

Optional filters:

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
python site_boundary_from_log_20260724.py finalize `
  "$site\01_boundary_candidates.csv" `
  --output "$site\01_boundary_final.csv" `
  --plot "$site\01_boundary_final.png" `
  --simplify-m 0.10

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

```bash
python3 site_coverage_planner_20260724.py preview \
  ~/field_plans/site_01/01_boundary_final.csv \
  --obstacles ~/field_plans/site_01/01_obstacles.csv \
  --output-dir ~/field_plans/site_01 \
  --angle-degrees 19 \
  --lane-spacing-m 0.90 \
  --stripe-end-trim-m 3.0 \
  --boundary-clearance-m 0.75 \
  --headland-passes 2 \
  --turn-radius-m 3.0 \
  --waypoint-spacing-m 0.50 \
  --ring-direction clockwise
```

Outputs:

- `02_plan_settings.json` — records every geometric and controller parameter.
- `02_coverage_segments.csv` — editable stripe checkpoint.
- `02_coverage_preview.png` — boundary, safe area, headlands, obstacles,
  numbered stripe arrows, and the preferred splice anchor.

### Parameters that must be measured/reviewed

| Parameter | Meaning |
|---|---|
| `lane-spacing-m` | Center-to-center cut spacing. The 42-inch deck is about 1.06 m; 0.90 m provides overlap, but verify actual cut width and GPS tracking. |
| `boundary-clearance-m` | Distance from the logged perimeter to the tractor reference point. Set it from deck/body overhang, survey meaning, GPS error, and desired safety margin. |
| `headland-passes` | Perimeter coverage passes before interior stripes. This creates working room but does not by itself guarantee a feasible U-turn. |
| `stripe-end-trim-m` | Distance removed from both ends of every clipped stripe to leave turning room. This replaces the notebook’s separate “shorten stripes” script and is visible in the editable CSV/plot. |
| `turn-radius-m` | Minimum planned Dubins radius and headland corner-rounding radius. The controller documentation estimates a 1.77 m steering limit; 3.0 m is intentionally gentler until field measurements justify less. |
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

### 4A. Build the candidate executable mission

```bash
python3 site_coverage_planner_20260724.py build \
  --settings ~/field_plans/site_01/02_plan_settings.json \
  --segments ~/field_plans/site_01/02_coverage_segments.csv \
  --output ~/field_plans/site_01/site_01_mission.txt
```

The builder:

1. Reconstructs the reviewed safe polygon.
2. Creates/densifies headland and stripe path pieces.
3. Tries all six Dubins path families for each transition.
4. Chooses the shortest connector that remains inside the safe polygon.
5. Refuses to write a mission if a connector or manual edit leaves that
   polygon.
6. Converts every local point back to lat/lon and writes the controller’s exact
   five-column format.

Additional outputs are:

- `site_01_mission_audit.csv`
- `site_01_mission_build_report.json`
- `site_01_mission_preview.png`

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

### 4C. Controller dry run and first field run

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
