# Bridgeville RTK Base Station — Setup & Rebuild Guide

## Overview
A solar-powered RTK GPS base station located in Bridgeville, PA. Transmits RTCM correction
data over ZeroTier VPN to rover units. Also logs solar charge controller data (Renogy Wanderer)
and ambient temperature via an ESP32.

---

## Hardware

| Component | Details |
|-----------|---------|
| Single board computer | Raspberry Pi 3 Model B |
| GPS receiver | u-blox ZED-F9P (simpleRTK2B Budget board) |
| Microcontroller | ESP32-S DevKitC 38-pin |
| Solar panel | ~100W panel |
| Battery | 12V lead-acid (Interstate) |
| Solar charge controller | Renogy Wanderer 10A |
| DC-DC converter | Matek UBEC DUO 4A (12V → 5V, replaces original TOBSUN unit) |
| Temperature sensor | DS18B20 on ESP32 GPIO32 |
| Serial adapter | TTL to RS232 module (ESP32 ↔ Renogy Modbus) |
| GPS antenna | Survey antenna with SMA connector |

### Power Notes
- Battery nominal: 12V, healthy range 12.5V–14.4V
- Converter output target: 5.0–5.2V
- Pi 3 draws ~600–800mA surge on boot; converter must handle transients
- Check `vcgencmd get_throttled` — `0x0` is clean; any non-zero means undervoltage has occurred
- Renogy Wanderer low-voltage disconnect: ~11.0–11.5V

---

## Network

| Item | Value |
|------|-------|
| Hostname | `rtkbase` |
| ZeroTier Network | `robotics_network` — ID `9f77fc393e0a16f8` |
| ZeroTier IP | `192.168.193.88` |
| ZeroTier node | `DDB182F9DE` |
| SSH access | `ssh al@rtkbase` or `ssh al@192.168.193.88` |

### WiFi Priority
NetworkManager manages WiFi (not wpa_supplicant directly). Two connection profiles are defined:

| Profile | SSID | Priority | Purpose |
|---------|------|----------|---------|
| `Pixel_4952` | Pixel_4952 | 10 | Field hotspot — preferred when in range |
| `preconfigured` | cui_bono | -1 | Home network — fallback |

On boot, the Pi connects to whichever known network is visible with the highest priority. If `Pixel_4952`
is in range it wins; otherwise falls back to `cui_bono` automatically. ZeroTier remains functional
through network switches.

```bash
# Verify WiFi profiles and priorities
nmcli connection show
nmcli connection show Pixel_4952 | grep -E "ssid|priority|autoconnect"
nmcli connection show preconfigured | grep -E "ssid|priority|autoconnect"

# Check current connection
iwconfig wlan0 | grep ESSID

# Add a new WiFi profile (e.g. rebuilding)
sudo nmcli connection add \
  type wifi \
  con-name "Pixel_4952" \
  ssid "Pixel_4952" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "YOUR_PASSWORD_HERE" \
  connection.autoconnect yes \
  connection.autoconnect-priority 10

sudo nmcli connection modify preconfigured connection.autoconnect-priority -1
```

---

## Repository

| Item | Value |
|------|-------|
| GitHub repo | `https://github.com/jones2126/tractor2025` |
| Local path | `/home/al/tractor2025` |
| Sparse checkout | Yes — tracks `RTKBase/` and `.gitignore` |
| RTK Base files | `RTKBase/Bridgeville/` |

### Sparse Checkout Setup (if rebuilding)
```bash
git clone https://github.com/jones2126/tractor2025
cd tractor2025
git sparse-checkout init
git sparse-checkout set --no-cone RTKBase/ .gitignore
```

---

## File Structure

```
/home/al/tractor2025/RTKBase/
├── Bridgeville/                        # Active production files
│   ├── rtcm_base_server_20260624.py    # Main RTCM server (runs as systemd service)
│   ├── logger_setup.py                 # Logging helper (copied from RTKBase/setup/)
│   ├── esp32_downloader_20260623.py    # ESP32 data download script
│   ├── esp32_timesync.py               # Boot-time ESP32 time sync script
│   ├── daily_esp32_download.sh         # Shell wrapper called by cron
│   ├── wifi_monitor_20260624.py        # WiFi RSSI monitor (runs as systemd service)
│   ├── daily_position_log.csv          # GPS position log (gitignored)
│   ├── current_position.json           # Latest GPS fix from F9P (gitignored, runtime)
│   ├── wifi_rssi_log.csv               # WiFi RSSI log, rolling 7 days (gitignored)
│   └── esp32-renogy-csv-logger/        # PlatformIO project for ESP32 firmware
│       └── src/rtkBaseESP32_csvLogger_20260623.cpp
├── setup/                              # Setup docs and shared utilities
│   ├── logger_setup.py                 # Logging helper (source — copy to Bridgeville/)
│   ├── rtcm_server.service             # systemd service file
│   ├── wifi-monitor.service            # systemd service file
│   ├── setup_ufw_rules.sh              # Firewall setup script
│   ├── README.md                       # System documentation
│   └── geodetic_to_ecef_and_back.md    # Reference: coordinate math
├── testing/                            # Archive and test projects
├── Brenham/                            # Placeholder for future Brenham base station
└── README.md

/home/al/scripts/
└── daily_esp32_download.sh             # Symlinked/called by cron (calls RTKBase/Bridgeville/ version)

/home/al/esp32_data/                    # Downloaded ESP32 CSV files (not in repo)
```

---

## Systemd Services

### rtcm_server.service
Runs the RTCM base station server continuously. Listens on TCP port 6001 (rovers) and
NTRIP port 2101 (mountpoint `/BASE`). Bound to `0.0.0.0` — accessible on all interfaces
including ZeroTier. Writes `current_position.json` every 60 seconds from live GGA fixes.

```ini
# /etc/systemd/system/rtcm_server.service
[Unit]
Description=RTCM Base Station Server
After=network-online.target
Wants=network-online.target
StartLimitBurst=5
StartLimitIntervalSec=300

[Service]
Type=simple
User=al
Group=al
WorkingDirectory=/home/al/tractor2025/RTKBase/Bridgeville
ExecStart=/usr/bin/python3 -u /home/al/tractor2025/RTKBase/Bridgeville/rtcm_base_server_20260624.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
Environment=PYTHONPATH=/home/al/tractor2025/RTKBase/Bridgeville

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable rtcm_server
sudo systemctl start rtcm_server
sudo systemctl status rtcm_server
journalctl -u rtcm_server -n 50 --no-pager
```

Note: `logger_setup.py` must be present in `RTKBase/Bridgeville/` for the server to start.
If missing: `cp ~/tractor2025/RTKBase/setup/logger_setup.py ~/tractor2025/RTKBase/Bridgeville/`

### esp32-timesync.service
Runs once at boot after NTP sync. Sends wall clock time to ESP32 so CSV records
have real timestamps instead of boot-relative milliseconds.

```ini
# /etc/systemd/system/esp32-timesync.service
[Unit]
Description=Sync wall clock time to ESP32 at boot
After=network-online.target time-sync.target
Wants=network-online.target time-sync.target
StartLimitIntervalSec=60
StartLimitBurst=2

[Service]
Type=oneshot
User=al
ExecStart=/usr/bin/python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_timesync.py
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable esp32-timesync
sudo systemctl start esp32-timesync
journalctl -u esp32-timesync --no-pager
```

### wifi-monitor.service
Monitors WiFi signal strength, logs to CSV, and sends ntfy.sh push notifications.
See WiFi Monitor section below for full details.

```ini
# /etc/systemd/system/wifi-monitor.service
[Unit]
Description=WiFi RSSI Monitor for RTK Base Station
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=al
Group=al
WorkingDirectory=/home/al/tractor2025/RTKBase/Bridgeville
ExecStart=/usr/bin/python3 -u /home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
Restart=always
RestartSec=30
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable wifi-monitor
sudo systemctl start wifi-monitor
sudo systemctl status wifi-monitor
journalctl -u wifi-monitor -n 30 --no-pager
```

---

## WiFi Monitor

`wifi_monitor_20260624.py` runs as a systemd service and provides:

- **Boot notification** via ntfy.sh: SSID, RSSI, signal label, lat/lon from `current_position.json`,
  and a Google Maps link. Fires on every service start (including `systemctl restart`).
- **5-minute RSSI polling**: logs `Timestamp, SSID, RSSI_dBm, Signal_Label` to `wifi_rssi_log.csv`
- **SSID change notification**: fires when the connected network changes (e.g. `cui_bono` ↔ `Pixel_4952`)
- **Weak signal notification**: fires once per weak episode when RSSI drops below -80 dBm;
  resets automatically when signal recovers
- **7-day rolling log**: old rows pruned on every poll cycle

| Setting | Value |
|---------|-------|
| ntfy topic | `rpi-rtkbase-jones2126` |
| Poll interval | 300 seconds (5 minutes) |
| Weak signal threshold | -80 dBm |
| Log file | `RTKBase/Bridgeville/wifi_rssi_log.csv` |
| Position source | `RTKBase/Bridgeville/current_position.json` |

Signal labels: **strong** ≥ -65 dBm, **medium** -65 to -80 dBm, **weak** < -80 dBm

Test ntfy manually:
```bash
curl -d "test message" https://ntfy.sh/rpi-rtkbase-jones2126
```

---

## Cron Jobs

```bash
crontab -e
```

```
# Daily ESP32 data download and delete at 8 PM
0 20 * * * /home/al/scripts/daily_esp32_download.sh >> /home/al/logs/esp32_cron.log 2>&1

# Push daily position log to GitHub at 8:15 PM
15 20 * * * /home/al/tractor2025/RTKBase/Bridgeville/push_daily_log.sh >> /home/al/logs/git_push.log 2>&1
```

---

## ESP32 Setup

### Device symlink
The ESP32 connects via USB and is accessed via a persistent symlink `/dev/esp32`.
Check or recreate the udev rule:
```bash
ls -la /dev/esp32
# Should point to /dev/ttyUSB0 or similar
cat /etc/udev/rules.d/99-esp32.rules
```

Typical udev rule:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32"
```

### Firmware
- PlatformIO project: `RTKBase/Bridgeville/esp32-renogy-csv-logger/`
- Current source: `src/rtkBaseESP32_csvLogger_20260623.cpp`
- Upload from Windows using PlatformIO
- After upload, the Pi's `esp32-timesync.service` must send `SETTIME` before CSV logging begins

### ESP32 Serial Commands
| Command | Action |
|---------|--------|
| `SETTIME <epoch>` | Set wall clock time |
| `STATUS` | Show file info, sync status, SPIFFS usage |
| `DOWNLOAD` | Download CSV data (non-destructive) |
| `DOWNLOAD_DELETE` | Download and clear CSV file |
| `CLEAR` | Delete CSV file and start fresh |
| `HELP` | List commands |

### Checking ESP32 status from Pi
```bash
cd /home/al/tractor2025/RTKBase/Bridgeville
python3 esp32_downloader_20260623.py status
python3 esp32_downloader_20260623.py download
python3 esp32_downloader_20260623.py download_delete
```

---

## ESP32 CSV Data Format

```
Timestamp,Avg_Temp_F,Battery_Voltage,Solar_Panel_Voltage,Solar_Panel_Amps,Load_Watts
2026-06-24 09:05:54 EST,78.27,14.40,17.90,0.83,4
```

- Logging interval: 60 seconds
- Max file size: 500KB before rotation
- Files stored on ESP32 SPIFFS (~1.3MB total)
- Downloaded daily to `/home/al/esp32_data/esp32_data_YYYYMMDD_HHMMSS.csv`
- Files older than 30 days are gzip compressed
- Files older than 365 days are deleted

---

## GPS / F9P

- Interface: USB serial via udev symlink `/dev/f9p` (see `/etc/udev/rules.d/99-rtk-devices.rules`)
- Mode: Fixed base station (Survey-In completed, coordinates stored)
- RTCM messages transmitted: 1005, 1074, 1084, 1094, 1230
- TCP port: 6001 (bound to 0.0.0.0 — accessible on all interfaces)
- NTRIP port: 2101, mountpoint `/BASE`
- Daily averaged position logged to `daily_position_log.csv` at 10:00 AM
- Current position written to `current_position.json` every 60 seconds (from live GGA)

### Checking RTCM server
```bash
sudo systemctl status rtcm_server
journalctl -u rtcm_server -n 50 --no-pager

# Check current GPS position
cat ~/tractor2025/RTKBase/Bridgeville/current_position.json
```

---

## Health Checks

```bash
# Power health
vcgencmd get_throttled        # 0x0 = clean
vcgencmd measure_temp         # CPU temp

# Services
sudo systemctl status rtcm_server
sudo systemctl status esp32-timesync
sudo systemctl status wifi-monitor

# WiFi
iwconfig wlan0 | grep ESSID
nmcli connection show

# GPS position
cat ~/tractor2025/RTKBase/Bridgeville/current_position.json

# WiFi RSSI log
tail -5 ~/tractor2025/RTKBase/Bridgeville/wifi_rssi_log.csv

# Disk / logs
ls -lh /home/al/esp32_data/
ls -lh /home/al/tractor2025/RTKBase/Bridgeville/gps_log.txt   # Watch for growth

# ZeroTier
sudo zerotier-cli status
sudo zerotier-cli listnetworks

# Previous boot undervoltage log
sudo journalctl -b -1 --no-pager | grep -i "voltage\|undervoltage"
```

---

## Known Issues / Notes

- `gps_log.txt` grows continuously while `rtcm_server` runs — monitor size and truncate periodically:
  ```bash
  sudo systemctl stop rtcm_server
  truncate -s 0 /home/al/tractor2025/RTKBase/Bridgeville/gps_log.txt
  sudo systemctl start rtcm_server
  ```
- `current_position.json` is written ~60 seconds after `rtcm_server` starts — not present
  immediately on a fresh boot. `wifi-monitor` boot notification will omit lat/lon if the file
  doesn't exist yet (service start order dependency).
- `logger_setup.py` must be present in `RTKBase/Bridgeville/` — it is not auto-deployed by
  sparse checkout since it lives in `RTKBase/setup/`. Copy it manually on a fresh build:
  `cp ~/tractor2025/RTKBase/setup/logger_setup.py ~/tractor2025/RTKBase/Bridgeville/`
- `wifi_rssi_log.csv` rolls to a 7-day window; pruning runs on every 5-minute poll cycle.
- SPIFFS is formatted on every ESP32 reboot (intentional — prevents corruption)
- ESP32 timestamps are boot-relative until `esp32-timesync.service` runs
- Pi 3 has no RTC — NTP must sync before `esp32-timesync.service` is useful
- `apt-daily-upgrade.timer` causes CPU spikes that can trigger undervoltage — consider disabling:
  ```bash
  sudo systemctl disable apt-daily-upgrade.timer
  sudo systemctl disable apt-daily.timer
  ```

---

## Python Dependencies

```bash
pip3 install pyserial pyubx2 --break-system-packages
```

---

*Last updated: 2026-06-24*
