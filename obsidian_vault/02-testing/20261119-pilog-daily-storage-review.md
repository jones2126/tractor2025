# Pi power/CPU logger: source discovery and daily-storage preparation

November 19, 2026. Installed by Al at13:00 EST; historical-log cleanup pending privileged operator command.

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

## Installed and independently verified

Al ran the approved migration. New daily CSV:
`/home/al/field_logs/2026-11-19/pilog_2026-11-19_130000.csv`.
First sample:5.04108V, throttling0x0, load1/load5=1.25/1.13, ARM1900MHz,
temperature41.7C. Installed script matches repository; timer remains active.
Backup: `/var/backups/pilog-migration-20261119T180000Z`.
Legacy archive: daily `pilog_legacy_archive_20261119T180000Z/pilog_legacy.tgz`.
Archive SHA256:
`6bfd5aa1ba6c98d553b17e8d0e2767e5c042bfe1aab1a07e0d0b94079c6cdf2a`.
Windows copy under ignored site `runs/20261119_pilog_legacy/`; archive and all
33 original file hashes independently verified. Privileged cleanup was attempted
but sudo requires the operator password; originals remain until PuTTY cleanup.
No services related to motion/sensor recording were restarted.

## Notification outage correction prepared

Al reported a missing boot voltage notice and requested the retry correction.
Review confirmed that CSV state was written before first notification, the old
curl call suppressed errors and later timer samples never retried. Startup
network-online ordering does not establish public internet availability. Journal
success does not prove ntfy acceptance; the actual missed-notice cause is unproven.

Updated source now separates pending/sent RAM markers, preserves the measured
report and sample timestamp, and attempts once per timer invocation with5s connect
and10s total timeouts. HTTP2xx plus curl success marks accepted; other outcomes
stay pending and appear in the journal. Data logging precedes network attempts.
An upgrade queues one honestly labelled catch-up sample because old delivery
state is unknown. Server acceptance is not confirmed client delivery; retries
following ambiguous responses may duplicate a notification.

Six isolated mocked Linux tests passed, including offline logging, midnight
retry, subsequent success/suppression, HTTP failure classes and upgrade behavior.
Activation helper `migrate_pilog.py --update-notifier` preserves the old script,
updates only the reviewed installed version, runs one sample and restores the
power timer. Pending operator sudo activation; no live code changed in preparation.
