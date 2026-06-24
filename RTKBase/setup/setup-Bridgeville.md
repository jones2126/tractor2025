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
- Ranegy Wanderer low-voltage disconnect: ~11.0–11.5V

---

## Network

| Item | Value |
|------|-------|
| Hostname | `rtkbase` |
| ZeroTier Network | `robotics_network` — ID `9f77fc393e0a16f8` |
| ZeroTier IP | `192.168.193.88` |
| ZeroTier node | `DDB182F9DE` |
| SSH access | `ssh al@rtkbase` or `ssh al@192.168.193.88` |

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
│   ├── rtcm_base_server_20260526.py    # Main RTCM server (runs as systemd service)
│   ├── esp32_downloader_20260623.py    # ESP32 data download script
│   ├── esp32_timesync.py               # Boot-time ESP32 time sync script
│   ├── daily_esp32_download.sh         # Shell wrapper called by cron
│   ├── daily_position_log.csv          # GPS position log (gitignored)
│   └── esp32-renogy-csv-logger/        # PlatformIO project for ESP32 firmware
│       └── src/rtkBaseESP32_csvLogger_20260623.cpp
├── setup/                              # Setup docs and shared utilities
│   ├── logger_setup.py                 # Logging helper (imported by rtcm server)
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
Runs the RTCM base station server continuously.

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
ExecStart=/usr/bin/python3 /home/al/tractor2025/RTKBase/Bridgeville/rtcm_base_server_20260526.py
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
```

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

- Interface: USB serial (appears as `/dev/ttyACM0` or similar)
- Mode: Fixed base station (Survey-In completed, coordinates stored)
- RTCM messages transmitted: 1005, 1074, 1084, 1094, 1230
- Server port: check `rtcm_base_server_20260526.py` for current TCP port

### Checking RTCM server
```bash
sudo systemctl status rtcm_server
journalctl -u rtcm_server -n 50 --no-pager
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
