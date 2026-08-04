# Field-testing analysis tools

Run these commands from the repository root in Windows PowerShell:

```powershell
Set-Location C:\Repos\tractor2025
```

Field-test evidence is stored under:

```text
field_testing\sites\<site>\runs\<run-id>\
```

Keep the downloaded source logs unchanged. Write generated analysis into an
`analysis` or similarly named subdirectory of the run.

## Python environment

The plotting tools require Folium, Matplotlib, NumPy, and pandas:

```powershell
python -m pip install -r .\field_testing\tools\requirements_field_testing_analysis_20260726.txt
```

## Recommended analysis for a completed or interrupted mission

`analyze_run_20260726.py` combines the 20 Hz Pure Pursuit log with the field
logger, mission audit, and boundary. It calculates time-weighted controller and
geometric cross-track error and creates a standalone interactive HTML map.

Repository-local run filenames do not have to share one timestamp. Supply the
actual files explicitly:

```powershell
$site = Join-Path $PWD 'field_testing\sites\62_Collins_polygon_1'
$run = Join-Path $site 'runs\20260803_135310'
$output = Join-Path $run 'analysis'

python .\field_testing\tools\analyze_run_20260726.py `
  --site-name 62_Collins_polygon_1 `
  --run-id 20260803_135310 `
  --run-dir $run `
  --output-dir $output `
  --pursuit-file "$run\pursuit_log_20260803_135312.csv" `
  --field-file "$run\62_Collins_polygon_1_20260803_135310.csv" `
  --mission-file "$run\62_Collins_polygon_1_mission.txt" `
  --audit-file "$site\62_Collins_polygon_1_mission_audit.csv" `
  --boundary-file "$site\01_boundary_final.csv" `
  --open-map
```

Outputs include an analysis JSON, embedded map-data JSON, and an interactive
HTML map. The analyzer defines Auto as field-logger `trans_mode=0` and only
uses the last observed field-logger mode, carried forward for at most 8.5
seconds. The bridge reports mode in bursts separated by as much as about eight
seconds; a 0.15-second nearest-record filter would discard most of the 20 Hz
pursuit path.

## Individual tools

| Tool | Use |
|---|---|
| `analyze_run_20260726.py` | Primary combined run analysis and interactive target-versus-actual map. |
| `analyze_pure_pursuit.py` | Quick pursuit-only tracking analysis, PNG error plot, HTML map, samples, and 5 m CTE bins. It cannot confirm radio mode. |
| `analyze_20260727_diagnostics.py` | Detailed straight-stripe, direction, geography, steering, and terminal-circling diagnostics for the older packaged run layout. |
| `build_straight_row_workbook_20260728.mjs` | Builds the detailed straight-row Excel diagnostic workbook after dependencies are configured. |
| `analyze_field_test_20260710.py` | General plots from one field-logger CSV. |
| `analyze_speed_20260710.py` | GPS-derived speed checks and plots. |
| `analyze_wifi_rssi_20260711.py` | Wi-Fi RSSI statistics and coverage plots. |
| `plot_mission_20260715.py` | Plots a mission before driving it. |
| `field_test_analysis_menu_20260726.ps1` | Legacy interactive collector/analyzer using `%USERPROFILE%\Documents\field_plans`. Prefer explicit repository-local commands for current runs. |
| `collect_site_run_20260724.ps1` | Downloads and verifies tractor logs and a mission package. Its default destination is the legacy Documents location. |
| `collect_rtkbase_esp32_20260724.ps1` | Downloads and verifies RTK-base ESP32 data. |
| `validate_perimeter_candidate_20260728.py` | Compares a candidate perimeter mission audit with the original. This is a planning validator, not a driven-run analyzer. |

## Quick pursuit-only command

Use this when only the pursuit log and mission are available:

```powershell
python .\field_testing\tools\analyze_pure_pursuit.py `
  "$run\pursuit_log_20260803_135312.csv" `
  "$run\62_Collins_polygon_1_mission.txt" `
  --out "$run\pursuit_analysis"
```

The `driving=True` field is only a controller state. Use the combined analyzer
when Auto-versus-Manual separation matters.

## Reviewing an interrupted run

Start with these questions:

1. At what time did `trans_mode` change from Auto (`0`) to Manual (`1`)?
2. Which `waypoint_idx` remained active before the intervention?
3. Did cross-track error grow while the waypoint index stopped advancing?
4. Did steering feedback follow `steer_setpoint`, or did `steer_pwm` saturate?
5. Was GPS still RTK Fixed with a valid heading?
6. Does the actual path remain inside the reviewed operating boundary?

Preserve the field CSV, pursuit CSV, mission, settings, validation, and bridge
journal together. Generated results can be reproduced; the source evidence
cannot.
