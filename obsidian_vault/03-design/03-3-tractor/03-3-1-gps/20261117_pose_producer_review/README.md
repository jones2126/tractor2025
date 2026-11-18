# Pose producer review — November 17, 2026

Design record. Al subsequently authorized preparation of the combined candidate,
`tractor_rpi/calc-publish-position-20261117.py`. It is not installed on Tractor01.
See `tractor_rpi/setup/CALC_PUBLISH_POSITION_20261117.md` for the field-test plan.
This design record itself grants no service or motion approval.

## Recorded direction

- Requested entry point: `calc-publish-position-20261117.py`.
- Al explicitly excludes `get_wifi_info()` from the replacement. Also omit its
  Pi-wlan0 polling thread and related diagnostic fields from the proposed pose
  schema. Remove imports/constants only when no remaining use exists.
- Keep GPS, IMU and encoder ingestion as distinct functions.
- Keep RTCM forwarding as its own responsibility.
- Background readers may block on their own input, but must not block estimation.
- Calculate one canonical pose and use explicit fan-out to independent consumers.
- Preserve raw GNSS quality independently of estimated-pose quality. Migration
  of existing consumers and field names needs a separate compatibility plan.

## Current navigation outputs

`current_navigation_udp_fields_20261117.csv` inventories **6002, 6009, 6010 and
6013**, the four RTCM-server navigation feeds discussed in this chat. It does not
claim to inventory the unrelated command/status/dashboard ports 6003, 6004,
6011, 6012, 6014 or 6015, nor TCP correction/router inputs.

All four receive the same encoded snapshot sent to localhost, nominally 20 Hz
per port. The CSV has 49 top-level fields plus one dynamic-child description for
`heading_nmea_types.<sentence_identifier>`, repeated for each port: 200 data rows.
The packet rate is the configured publication target, not a fresh measurement
rate or delivery guarantee. Source-refresh descriptions identify approximately
5 Hz GNSS data, 1 Hz satellite polls, event-driven fields and retained values.
Both NAV-PVT and NAV-SAT can update a shared satellites-used field, each polled
at 1 Hz, so that field can update twice per second.

CSV columns include consumer, type, units, field refresh rate, meaning,
freshness/duplication caveats, proposed treatment and source declaration line.
Wi-Fi fields remain in this **current** inventory and are marked for removal
from the future producer. Dynamic dictionary children depend on observed NMEA
sentence identifiers; the wildcard is not an additional literal top-level key.

Source: `tractor_rpi/rtcm_server_20260828.py`, inspected locally. Schema and rates
were read from source rather than measured again on Tractor01. Source SHA-256
and verification are recorded in `inventory_provenance.json`.

## Reviewed execution split — implemented in the uninstalled candidate

One main thread owns the estimated pose. Separate background readers own the
device inputs. The three main-loop ingestion functions consume queued readings;
they do not reopen or reread devices already owned by the workers.

| Main-loop function | Work |
|---|---|
| `ingest_gps()` | Consume new base/heading readings and source quality, normalize timestamps and identify genuinely new measurements. |
| `ingest_imu()` | Consume every queued IMU sample, retaining sample times and calibration/error state. |
| `ingest_encoders()` | Consume both sides' increments/counts, align their times and identify resets, gaps and invalid reads. |
| `update_pose()` | Process the acquired measurement batch in measurement-time order, propagate pose and apply permitted trusted corrections. |
| `assess_pose_quality()` | Calculate source freshness, discontinuities, uncertainty and validity. This does not authorize motion. |
| `publish_navigation_if_due()` | Copy one consistent pose/source/quality snapshot to configured recipient endpoints, initially targeting 20 Hz. |
| `queue_diagnostics_if_due()` | Enqueue a small diagnostic snapshot; perform no blocking network/subprocess/log-file work in the estimator loop. |

A nominal 20 Hz main-loop target is proposed initially to match navigation
publication. It is subject to timing/load validation. IMU acquisition stays at
its own configured rate (currently 104 Hz nominal); several IMU samples may be
processed in one main-loop pass. Integration uses actual sample times, not an
assumed 50 ms interval or just the most recent gyro reading. The five-second
GNSS correction concept is a separate correction policy, not the loop rate.

| Background thread function | Ownership and role |
|---|---|
| `read_gps_base()` | Sole base-link serial reader, including NMEA and satellite replies; schedules diagnostic polls. |
| `read_gps_heading()` | Sole heading serial reader, including RELPOSNED and satellite replies. |
| `read_imu()` | Sole IMU acquisition worker; timestamp and queue samples. |
| `read_encoder(side)` — left instance | Own left encoder serial input and queue readings/errors. |
| `read_encoder(side)` — right instance | Own right encoder serial input and queue readings/errors. |
| `forward_rtcm_corrections()` | Maintain correction TCP connection and forward bytes to the base-link receiver. |
| `diagnostics_worker()` | Write queued diagnostic records and handle slow housekeeping separately. No Wi-Fi polling in this producer. |

RTCM forwarding is a continuous worker, not also called from the main loop.
Correction writes and base-receiver diagnostic poll writes must use the same
serial-write lock, preserving the current frame-interleaving protection.
There remains exactly one reader for each serial input. This program must never
claim the main control `/dev/teensy`; wheel encoders are separate USB devices.

Device setup, calibration startup, thread creation and cleanup happen outside
the steady-state main loop. Use bounded queues and detect overflow; silently
dropping wheel increments or gyro samples is unacceptable. Reader errors,
device resets and encoder recovery re-baselines must reach the estimator as
discontinuities. Device `millis` and Pi monotonic time require explicit alignment.
Diagnostic queues may shed records with an explicit counter; sensor loss must
affect estimate validity rather than silently appearing healthy.

This design proposes seven background workers plus one main thread. Threads
help separate waits; they do not by themselves prove lower CPU load. Measure
loop latency, queue depths, missed samples and CPU on the RPi before adoption.

## Decisions still open

- Pose message schema beyond the first non-authoritative capture candidate.
- Measured scheduling/processing budget and queue limits.
- Encoder scale, cross-side discrepancy, timestamp synchronization and gap handling.
- GNSS measurement-epoch alignment, IMU bias handling and correction policy.
- Estimator uncertainty model and acceptance tests.
- Ownership of degraded-motion time/distance enforcement remains a separate
  safety-arbitrator discussion. Existing freshness/lease/watchdog limits remain
  unchanged; publication of a pose estimate never grants motion by itself.
