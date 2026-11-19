# Pi power/CPU logger: source discovery and daily-storage preparation

November 19, 2026. Prepared replacement; live installation pending approval.

Read-only SSH located `pilog.service` and `pilog.timer` under
`/etc/systemd/system/`. Service runs `/usr/local/bin/pilog.sh` as root, oneshot,
Nice15, timeout120s. Timer: OnBootSec30s, OnUnitActiveSec2min, AccuracySec10s.
Timer was enabled/active. CSVs currently live in `/var/log/pilog`; the current
boot continued appending a file initially created November18. No source logs
were moved, truncated or deleted during this investigation.

Captured source is `tractor_rpi/setup/pilog/installed_pilog_20261119.sh.txt`.
Fields: time, uptime, Pi5V rail, throttling bitmask, load1/load5, ARM MHz and CPU
temperature. Load is not CPU-utilization percentage. Voltage is Pi5V, not JRK VIN.
The script sends a first-sample boot ntfy and has no repeated runtime-alert
branch. Repeated alerts reported previously therefore remain separately untraced.

Prepared `tractor_rpi/setup/pilog/pilog.sh` changes only storage handling and
explicit timezone, preserving measurement commands, fields, cadence and curl
boot-notice retry settings. Output becomes
`/home/al/field_logs/YYYY-MM-DD/pilog_YYYY-MM-DD_HHMMSS.csv`, using Eastern dates,
next to raw telemetry segment directories. `/run/pilog.current` continues tracking
the active file. Each sample rechecks the day; midnight opens another CSV without
another boot notice. Switching from the old path midboot likewise does not repeat
the notice. New daily directories are owned by al, so root logging cannot block
the user-run recorder. Historical logs stay in place.

Three isolated tests passed on Tractor01 using temporary files and mocked
sensor/date/notification commands: append/one boot notice, midboot migration
preserving old bytes, and midnight rotation without another notice. Bash syntax
check also passed. No installed script/unit or production log was changed.

Installation should back up the installed script/hash, avoid overlapping an
active pilog invocation, install only the reviewed script, and validate one new
sample. No bridge/GNSS/recorder restart or daemon-reload is required. The next
timer invocation uses the new script. Keep the existing RAM state file.

Collection follow-up: frozen collector defaults to `/var/log/pilog`; use explicit
daily-folder input for future logs, handling both dates for overnight runs. Manual
archive selections must include the sibling pilog CSV as well as telemetry
segments. No changes to frozen collectors or sealed raw segments were made.
