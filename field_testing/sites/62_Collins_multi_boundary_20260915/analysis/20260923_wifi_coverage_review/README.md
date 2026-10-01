# 62 Collins Wi-Fi coverage review

This evidence layer combines the revision-2 site inventory with the September 15 multi-area field log and four September 23 field telemetry logs. Open `62_Collins_wifi_coverage_review_20260923.html` to switch between tractor-router and Pi wlan0 RSSI and between manual and auto samples.

## Finding

The recorded tractor-router values were no worse than -72 dBm during active manual/auto rows. That is encouraging, but it does not prove uninterrupted connectivity because the logger stores the last router value without a receive timestamp or age. Across the included sessions, the Pi wlan0 field had 7 active-mode unavailable intervals totaling 60.62 seconds.

This folder is analysis output only. It does not change the authoritative site inventory or create a launchable mission. See `wifi_coverage_report_20260923.json` for source hashes, per-run statistics, missing intervals, limitations, and recommended instrumentation.
