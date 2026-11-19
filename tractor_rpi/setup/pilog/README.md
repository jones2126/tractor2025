# Pi power/resource log storage

November 19, 2026: installed source located at `/usr/local/bin/pilog.sh`.
`installed_pilog_20261119.sh.txt` preserves its original source. The revised
`pilog.sh` is prepared for a separately approved installation, not yet deployed.

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
