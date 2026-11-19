# Pi power/resource log storage

November 19, 2026: installed source located at `/usr/local/bin/pilog.sh`.
`installed_pilog_20261119.sh.txt` preserves its original source. The revised
`pilog.sh` was installed November19 at13:00 EST and its first sample verified.

Existing timer: `/etc/systemd/system/pilog.timer`, first sample about 30 seconds
after boot, subsequent samples every 2 minutes, timer accuracy 10 seconds.
`pilog.service` is root-run, Type=oneshot, Nice=15, TimeoutStartSec=120.
No change to units, cadence, hardware commands or notification retries is needed.

New output: `/home/al/field_logs/YYYY-MM-DD/pilog_YYYY-MM-DD_HHMMSS.csv`.
Dates/times explicitly use America/New_York. It is a sibling of sealed telemetry
segment directories; never append to a checksummed closed telemetry segment.
The script creates new daily directories owned by al so the recorder remains
able to write. Root-owned CSV files remain readable. `/run/pilog.current` records
the active CSV and clears on reboot. Date changes create a new daily CSV without
another boot notice. Midboot migration from the old path likewise preserves
the old file and does not falsely announce a new boot.

Unchanged columns:
`time,uptime_s,ext5v_v,throttled,load1,load5,arm_mhz,temp_c`.
Load averages are runnable/uninterruptible task demand, not CPU utilization
percent. ARM MHz is CPU frequency, not utilization. EXT5V is Pi supply voltage,
not JRK battery VIN. No new measurement is added in this storage-only change.

The captured script sends an ntfy boot notice once per boot, with voltage,
throttling flags, load, clock, temperature and log path. Its curl retries remain
unchanged. Repeated in-operation power alerts reported by Al have not been
traced to this script; it contains no such repeated-alert branch.

Keep historical `/var/log/pilog/` files untouched. The frozen evidence collector
defaults there; for new daily captures it needs an explicit `--pilog-dir` pointing
to the relevant daily folder (or a separately reviewed migration). Midnight runs
may require both dates; do not assume the old collector discovers daily folders.
The manual archive command that names only telemetry segments/preflight also
needs to include the sibling pilog CSV explicitly when collecting future runs.

Before installation: preserve the installed script and its SHA-256 in a dated
backup, verify source identity and clean main revision, install only this script
at `/usr/local/bin/pilog.sh` mode0755, then validate one sample. No sensor/bridge
service restart or unit daemon-reload is required; the next timer invocation uses
the new script. Do not overlap an active invocation. Retain `/run/pilog.current`
so a midboot migration does not send another boot notice.

Isolated Linux checks (mock sensor/date/notice commands; temporary output only):
`python3 tractor_rpi/setup/pilog/test_pilog.py`.

## Approved migration helper

Al approved installation and archival/removal of historical CSV originals on
November19. Administrative access needs the operator's PuTTY sudo password;
noninteractive SSH cannot perform that privileged step. Run from the clean,
synchronized production checkout:

```bash
cd /home/al/tractor2025 && sudo python3 tractor_rpi/setup/pilog/migrate_pilog.py --apply
```

The helper briefly pauses only pilog.timer, refuses overlapping samples, backs up
the installed script/state, creates and internally verifies a legacy-log archive
in the daily field folder, installs the replacement and validates one sample.
It restores the timer. Legacy CSV originals remain until the archive is copied
off-Pi and independently verified. Cleanup is then an explicit helper phase with
the manifest path and independently verified SHA256; it refuses changed files
and removes only the archived, matching legacy files and empty legacy directory.

## Boot-notice retry correction — prepared, not installed

The storage-only installed version lost the boot notice if its initial curl
retry window expired before internet access arrived: CSV state suppressed all
later notice attempts. It also suppressed errors and did not reject HTTP failures.

The correction keeps `.notice-pending` and `.notice-sent` beside the RAM CSV
state. It retains the initial measured sample/message and retries once on every
subsequent two-minute timer invocation, with5s connect/10s total timeouts.
Only successful HTTP2xx acceptance marks sent; DNS/connect/HTTP failures remain
pending and are logged in the service journal. Midnight does not clear the notice.
Upgrade during a running boot queues one catch-up report because old delivery
success cannot be inferred; its label gives the actual sample time, not a claimed
boot time. Acceptance is not proof of phone delivery; an ambiguous HTTP result
can cause a retry/duplicate. CSV logging always occurs before the network attempt.

Six isolated checks passed, including repeated network failure, retry after
midnight, later success/no further attempts, HTTP403/429/503/302 rejection and
upgrade catch-up. No real notifications were sent by these tests.

Install only this reviewed update, preserving backup and existing CSV/state:
`sudo python3 tractor_rpi/setup/pilog/migrate_pilog.py --update-notifier`.
The helper briefly pauses the power timer, refuses an overlapping sample,
installs only over the reviewed prior source, takes one sample, reports notice
pending/accepted, and restores the timer. Other services are untouched.
