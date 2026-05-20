# Teensy 4.1 — Ethernet Data Logger (ROSBAG-style)

**Hardware:** Teensy 4.1 with built-in SD card + RJ45 Ethernet add-on board
**Goal:** Log robot sensor data to SD card in CSV format; download via web browser on demand
**Status:** All three phases complete and tested ✓ (2026-05-17)

---

## Concept

Mimic ROSBAG behavior without ROS:
- Main loop collects sensor data at a fixed rate and writes rows to a `.csv` file on the SD card
- Teensy hosts a small HTTP server on the local network
- Browse to the Teensy's IP address to see a file list and download logs

---

## Hardware Notes

| Item | Detail |
|------|--------|
| Teensy 4.1 | Built-in micro SD slot (uses dedicated bottom pads, no edge pin conflict) |
| Ethernet add-on | RJ45 board soldered to Teensy 4.1 **bottom pads only** — does NOT use any numbered edge pins (0–41) |
| Library | QNEthernet by ssilverman (`ssilverman/QNEthernet@^0.32.0`) |
| IP assignment | DHCP — confirmed working, assigned 192.168.1.215 |

### Ethernet Pin Compatibility — Confirmed via PJRC Pinout Card
The Ethernet add-on uses bottom pads exclusively. **All standard edge pins (0–41) remain free.**
This means Ethernet is fully compatible with the NRF24 radio (pins 9–13) on the same Teensy 4.1.

> An earlier analysis incorrectly claimed Ethernet used pin 13. This was wrong — confirmed by PJRC pinout card which shows pin 13 = SCK/LED only. See [[Teensy41_Pin_Reference]] for full pin documentation.

---

## Phase 1 — IP Address Test ✓

**Goal:** Minimal sketch to confirm Ethernet hardware is working.
- [x] QNEthernet library via PlatformIO lib_deps
- [x] DHCP IP obtained and printed to Serial
- [x] Simple web page served at IP address in browser

**Code:** `tractor_rpi/testing/teensy41_ethernet_phase1/`

---

## Phase 2 — SD Card CSV Logging ✓

- [x] SD card initialized using `BUILTIN_SDCARD` (no extra wiring)
- [x] Auto-numbered log files: `log001.csv`, `log002.csv`, ...
- [x] Header row written on startup
- [x] 10 Hz logging (one row every 100 ms), non-blocking
- [x] Flush to disk every 50 rows (~5 seconds) to protect against power loss
- [x] Web status page shows log filename, rows written, uptime

**Code:** `tractor_rpi/testing/teensy41_ethernet_phase2/`

---

## Phase 3 — Web Download ✓

- [x] HTTP server routes requests by URL path
- [x] `GET /` — status card + table of all CSV files on SD with size
- [x] `GET /log001.csv` — streams file to browser as download
- [x] `GET /?delete=log001.csv` — deletes file, redirects back to `/`
- [x] Active log file tagged "logging" — protected from delete
- [x] Works in both Chrome and Edge

**Code:** `tractor_rpi/testing/teensy41_ethernet_phase3/`

---

## Lessons Learned — HTTP Gotchas

These took multiple iterations to resolve:

| Issue | Fix |
|-------|-----|
| SD library can't open the same file twice | Close active log before reading it for download; reopen after |
| Browser saved HTML instead of CSV | `SD.exists()` fails while log is open → routing fell through to page handler; fixed by routing any `.csv` URL directly to file handler |
| Chrome made 6 parallel connections | Added `Accept-Ranges: none` header — tells Chrome not to split downloads into parallel range requests |
| Edge would not download | Same `Accept-Ranges: none` fix resolved Edge as well |
| Explicit `\r\n` in headers | Replaced `println()` with `print("\r\n")` for strict HTTP compliance |

---

## CSV Format (current — simulated fields)

```
timestamp_ms, steering_val, transmission_bucket, gps_status, estop, uptime_s
```

**Next step:** Replace simulated values with real sensor reads from the tractor main loop (steering pot A9, transmission bucket, GPS status from bridge, radio signal quality).

---

## Phase 4 — SD Management + Date/Time ✓ (2026-05-19)

**Code:** `tractor_rpi/testing/teensy41_ethernet_phase4/`

### SD Card Management
- [x] **Delete All button** on web page → `GET /?deleteall` → closes active log, deletes all CSV files, reopens fresh log
- [x] After delete all, next log uses timestamp name if RTC is set, otherwise resets to log001.csv
- Full SD reformat: NOT implemented — "Delete All" is the safe equivalent; SdFat format() is risky on a live system

### Date/Time (Teensy 4.1 built-in RTC)
Teensy 4.1 has a built-in RTC that holds time between power cycles with a coin cell battery.

**Three-layer sync — fully automatic, no user action needed:**

| Priority | Source | Method | Requirement |
|----------|--------|--------|-------------|
| 1 | Hardware RTC | Read on boot via `Teensy3Clock.get()` | Coin cell + previously set |
| 2 | NTP | UDP to `time4.google.com` (216.239.35.4) — no DNS needed | Internet access through router |
| 3 | Browser JS | Page load silently calls `GET /?settime=UNIX_TIMESTAMP` | Any browser visit |

- If RTC was previously set and coin cell is alive → time is available immediately at boot, no network needed
- NTP attempted after DHCP with 5-second timeout
- Browser fallback: JavaScript on every page load sends `Date.now()` — works on local network with no internet
- Web page status row shows **green** (RTC set) or **red** (not set)

### Timestamped Log Filenames
- RTC set → `20260519_143052.csv` (YYYYMMDD_HHMMSS)
- RTC not set → `log001.csv` (sequential fallback)

Much more useful in the field — filename tells you exactly when the run happened.

---

## Notes

