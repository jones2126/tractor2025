# Heading F9P satellite and heading comparison field test — 2026-08-29

## Purpose

Determine whether Heading-F9P satellite availability or signal strength is
associated with moving-baseline carrier changing from `fixed` to `float`, and
compare the F9P heading with heading calculated from the Base-Link RTK
latitude/longitude track.

The 2026-07-27 log did not record Heading-F9P satellite counts, so this test
cannot recreate that missing value. It establishes a complete, repeatable
baseline for the next comparison.

## Files used

- `tractor_rpi/rtcm_server_20260828.py`
- `tractor_rpi/field_test_logger_20260828.py`
- `tractor_rpi/setup/rtcm-server-tractor01.service`
- `field_testing/tools/analyze_heading_f9p_20260828.py`

## 1. Before going to the tractor — development machine

Commit and push the new files to GitHub. Confirm that `git status` contains
only the intended files before committing.

```powershell
cd C:\Repos\tractor2025
git status
git add tractor_rpi/rtcm_server_20260828.py `
        tractor_rpi/field_test_logger_20260828.py `
        tractor_rpi/setup/rtcm-server-tractor01.service `
        tractor_rpi/setup/rtcm-server.service `
        tractor_rpi/setup/install_services.sh `
        tractor_rpi/testing/configure_heading_f9p_20260727.py `
        tractor_rpi/testing/test_gps_satellite_logging_20260828.py `
        field_testing/tools/analyze_heading_f9p_20260828.py `
        obsidian_vault/02-testing/20260829-heading-f9p-satellite-field-test.md
git commit -m "Log per-receiver F9P satellite diagnostics"
git push
```

## 2. Synchronize tractor01 and install the service definition

Put the tractor in Pause, leave the blades off, and connect by SSH. If the
hostname does not resolve, use the tractor's current IP or ZeroTier address.

```bash
ssh al@tractor01
sudo systemctl stop rtcm-server.service
cd /home/al/tractor2025
git pull --ff-only
sudo install -m 0644 tractor_rpi/setup/rtcm-server-tractor01.service /etc/systemd/system/rtcm-server.service
sudo systemctl daemon-reload
sudo systemctl restart rtcm-server.service
sudo systemctl status rtcm-server.service --no-pager
```

The field logger is not a service. It is started manually for each test, so no
field-logger `.service` file needs changing.

## 3. Verify the new GPS fields before moving

First check that the service is not reporting serial or checksum errors:

```bash
journalctl -u rtcm-server.service -n 50 --no-pager
```

Then display one UDP 6009 packet:

```bash
python3 - <<'PY'
import json, socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 6009))
s.settimeout(3)
d, _ = s.recvfrom(8192)
p = json.loads(d)
for name in (
    "base_numSV_used", "base_numSV_visible", "base_cno_mean_dbhz",
    "heading_numSV_used", "heading_numSV_visible", "heading_cno_mean_dbhz",
    "carrier", "headValid", "heading_deg",
):
    print(f"{name}: {p.get(name)}")
PY
```

Do not begin the run if all Base-Link or all Heading satellite fields remain
`None`. One missing packet immediately after startup is acceptable; persistent
missing values indicate the poll responses are not reaching the server.

## 4. Start a dedicated field log

```bash
mkdir -p /home/al/field_logs/20260829_heading_satellite
python3 /home/al/tractor2025/tractor_rpi/field_test_logger_20260828.py \
  --output /home/al/field_logs/20260829_heading_satellite/field_test_20260829_heading_perimeter.csv
```

Leave this terminal visible. Confirm that its row count increases.

## 5. Drive the test

1. Park at the selected start/finish location with a clear view of the sky.
2. Remain completely stationary for at least 20 seconds; 25 seconds gives a
   little margin for automatic pause detection.
3. Select Manual and drive one perimeter lap at a steady speed. Use the same
   route, direction, antenna arrangement, and approximate speed as the earlier
   run when practical.
4. Avoid stopping during the lap unless safety requires it. The analyzer uses
   the first two long stationary pauses to identify the lap.
5. Return to the starting point, select Pause, and remain stationary for at
   least 20 seconds; again, 25 seconds is preferred.
6. Stop the logger cleanly with Ctrl+C and record the filename.

For a clockwise/counterclockwise comparison, make each direction a separate
CSV with its own start and finish pauses. Separate files keep automatic lap
selection unambiguous.

## 6. Copy the CSV to the development machine

Run this from PowerShell on the development machine:

```powershell
$runDir = 'C:\Repos\tractor2025\field_testing\sites\62_Collins_polygon_1\runs\20260829_heading_satellite'
New-Item -ItemType Directory -Force -Path $runDir
scp 'al@tractor01:/home/al/field_logs/20260829_heading_satellite/*.csv' $runDir
```

## 7. Analyze on the development machine

The analysis should run on the development machine, not tractor01. It needs
only Python's standard library and the field CSV.

```powershell
python C:\Repos\tractor2025\field_testing\tools\analyze_heading_f9p_20260828.py `
  --input C:\Repos\tractor2025\field_testing\sites\62_Collins_polygon_1\runs\20260829_heading_satellite\field_test_20260829_heading_perimeter.csv
```

The analyzer automatically:

- selects the motion between the two stationary pauses;
- calculates direction of travel from RTK positions over a 3 m baseline;
- rejects turns greater than 12 degrees over that baseline;
- requires `RTK Fixed`, `head_valid=true`, and speed at least 0.25 m/s;
- compares F9P heading with position-derived heading;
- reports heading error and satellite statistics by carrier state; and
- lists every `fixed`, `float`, or `none` carrier episode and duration.

It writes:

- `heading_analysis_summary_20260828.json`
- `heading_comparison_samples_20260828.csv`

If automatic pause selection fails, rerun with explicit limits:

```powershell
python C:\Repos\tractor2025\field_testing\tools\analyze_heading_f9p_20260828.py `
  --input <csv-path> --start-elapsed 25 --end-elapsed 240
```

## 8. Interpretation checklist

Compare `fixed` and `float` periods for:

- Heading satellites used and visible;
- Heading mean/minimum C/N0;
- Base-Link satellites over the same seconds;
- whether only Heading degrades or both receivers degrade together;
- F9P-versus-position heading error; and
- physical location on the perimeter, especially near trees, buildings, or
  other obstructions.

If Heading satellite counts and C/N0 stay steady during float, investigate
configuration, antenna cable/ground plane, RTCM moving-base messages, or the
Heading receiver's correction stream rather than simple satellite visibility.
