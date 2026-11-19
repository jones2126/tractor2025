# November 19 manual IMU/GNSS/encoder recording run

Operator-reported times use America/New_York (EST, UTC-5). Raw telemetry and
independent checksum verification are complete; do not treat these notes
as measured maneuver boundaries or conclusions about pose accuracy.

## Preparation

Fresh Pi boot. Al moved the tractor to an area with fewer perceived multipath
concerns; whether this explains improved GNSS is unproven. Initial preflight
showed Fixed position/heading and live right encoder/IMU, but silent left encoder.
Identity-guarded left USB reauthorization was instructed based on November 18's
demonstrated recovery; subsequent report confirmed both encoder streams live.

After starting the position service, full preflight passed, report:
`/home/al/field_logs/2026-11-19/preflight_105734_413335.txt`.
Position Fixed, Heading Fixed in 100% of sampled snapshots, baseline 1.0850 m,
accuracy 0.541 degrees; 30 Heading satellites used. IMU calibrated/aligned,
both encoders 50 distinct records/5 s, controls Pause/neutral 2836, v5 firmware.
Recorder session `e6702657a90e4dbbbc1009fa4f69db97`, error=None/losses=0,
approximately 31.9 GB free. Current segment at preflight was
`/home/al/field_logs/2026-11-19/telemetry_154324_711636Z`; subsequent segments rotate.

## Operator event notes

| EST | UTC | Report |
|---|---|---|
| 10:59 | 15:59 | Engine not running; checking manual steering, then returning to Pause |
| 11:01 | 16:01 | Engine running after two start attempts |
| Approximately 11:14 | Approximately 16:14 | Manual drive complete; parked, engine off, in Pause |

Al reported ten-second pauses between CW/CCW squares and hard turns. Exact number,
order and times of hard-turn pauses should be reconciled with recorded motion;
do not assume every inferred stationary episode is an intended marker. Today's
drive percentages were not supplied; do not copy November 17's percentages as
today's measured commands.

## Collection and initial replay complete

Downloaded archive `manual_run_20261119.tgz`, 40,090,094 bytes, SHA-256
`eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19`.
Windows independently verified the archive and all three closed segment manifests;
consumed channel counts match metadata. Preserved ignored raw copy:
`field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual/`.
Original Tractor01 files were not changed or deleted.

Replay: `analysis/20261119_manual_sensor_replay/replay_20261119_manual.html`
under the same site directory. Uses the preceding IMU replay's layout and shared
mission map/style, adding wheel, steering, manual drive and JRK VIN charts.
Default requested window is 10:57–11:16 EST; first IMU row is 10:57:13.
120,742 raw IMU rows become 11,308 playback points with bin extrema.
Offline JavaScript playback/control checks passed. Maneuver dimensions and encoder
scale/slip analysis remain pending; the replay does not establish navigation accuracy.

Reported 11:01–11:14 window: IMU 83,574 rows (maximum source gap 0.01321 s),
left encoder 7,800 (maximum 0.10345 s), right encoder 7,773 (maximum 0.10448 s).
GNSS recorder arrival maximum gap 0.20846 s; status maximum 0.18280 s. These are
host/source transport observations, not proof of independent hardware sampling.

### Preserved collection procedure

Close current segment without interrupting continuous recording, identify all
segments covering preparation and the run, verify manifests and copy raw files
off Tractor01. Preserve originals. Retain producer calibration/restart events,
full GNSS, native IMU, both raw encoders, status/controller/JRK, router data and
the preflight report. An archive must have its own SHA256 and an independently
verified off-Pi copy. No cleanup is authorized.

Replay/analysis should separate operator timestamps from inferred boundaries,
healthy GNSS reference windows from degraded intervals, raw encoder counts from
uncalibrated provisional distances, and observed antenna-path turn dimensions
from whole-tractor planning clearance. No inferred policy is field validated merely
because the manual capture completes.
