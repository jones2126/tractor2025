# Firmware Version Log

Tracks what is actually running on each device. The repo contains the source;
this file records which file and commit is active on each device.

**Rule:** Update this file immediately after every flash or deploy, then commit and push.

Older firmware versions archived in `tractor_teensy/src/archive/` and `radiocontrol_nrf24radio/src/archive/`
RPi scripts updated via `git pull` on boot (see `repo-sync.service`).
Full setup details in `tractor_rpi/setup/tractor01_rpi_setup.md` and `tractor02_rpi_setup.md`.

---

## Current Production Files

`{date}` in the File Name column represents the actual date stamp used in the filename (e.g. `_20260617`).

| File Name | Runs On | Last Updated | Commit | Source File Raw URL |
|-----------|---------|-------------|--------|---------------------|
| `teensy_main_{date}.cpp` | Teensy 4.1 — tractor01 | 2026-06-17 | 9c585d1 | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_teensy/src/teensy_main_20260617.cpp |
| `teensy_main_{date}.cpp` | Teensy 4.1 — tractor02 | never | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_teensy/src/teensy_main_20260609.cpp |
| `RadioControlNRF24_{date}.cpp` | Teensy 3.2 — RC handheld | 2026-06-12 | 9cd39f7 | https://raw.githubusercontent.com/jones2126/tractor2025/main/radiocontrol_nrf24radio/src/RadioControlNRF24_20260612.cpp |
| `rtcm_server_{date}.py` | tractor01 RPi5 | 2026-06-17 | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/rtcm_server_20260617.py |
| `rtcm_server_x20d_{date}.py` | tractor02 RPi5 | 2026-06-17 | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/rtcm_server_x20d_20260617.py |
| `teensy_serial_bridge_{date}.py` | tractor01 RPi5, tractor02 RPi5 | 2026-03-10 | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/teensy_serial_bridge_20260310.py |
| `led_status_controller.py` | tractor01 RPi5, tractor02 RPi5 | — | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/led_status_controller.py |
| `field_test_logger_{date}.py` | tractor01 RPi5, tractor02 RPi5 | 2026-06-17 | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/field_test_logger_20260617.py |
| `rtcm_base_server_{date}.py` | RTK Base — Brenham RPi3 | — | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/tractor_rpi/rtcm_server_20260617.py |
| `rtcm_base_server_{date}.py` | RTK Base — Bridgeville RPi3 | 2026-05-26 | — | https://raw.githubusercontent.com/jones2126/tractor2025/main/BridgevilleRTKBase/raspberry-pi/production/rtcm_base_server_20260526.py |

---

## Flash History

| Date | Device | File | Commit | Notes |
|------|--------|------|--------|-------|
| 2026-06-17 | Teensy 4.1 — tractor01 | `teensy_main_20260617.cpp` | 9c585d1 | Fixed e-stop relay pin (ESTOP_RELAY_PIN = 30) |
| 2026-06-12 | Teensy 3.2 — RC handheld | `RadioControlNRF24_20260612.cpp` | 9cd39f7 | First commit using this tracking process |
| 2026-06-14 | — | — | — | firmware_versions.md created; prior flash history not recorded |

> All entries above this line predate this tracking file.
> From this point forward, record the commit hash at every flash or deploy.

---

## Flash Workflow — Teensy Firmware

**Step 1 — Make sure source is current:**
```bash
cd ~/tractor2025
git pull origin main
```

**Step 2 — Note the commit you are about to flash:**
```bash
git rev-parse --short HEAD
```

**Step 3 — Flash via PlatformIO** (Teensy must be connected via USB):
```bash
# Tractor Teensy 4.1
cd ~/tractor2025/tractor_teensy
pio run --target upload

# RC handheld Teensy 3.2
cd ~/tractor2025/radiocontrol_nrf24radio
pio run --target upload
```

**Step 4 — Update the Current Production Files table above, then commit:**
```bash
cd ~/tractor2025
git add obsidian_vault/04-reference/firmware_versions.md
git commit -m "firmware_versions: flash <device> at <hash>"
git push origin main
```

---

## Deploy Workflow — RPi Python Services

RPi scripts are pulled automatically at boot via `repo-sync.service`.
To force an immediate update:
```bash
sudo systemctl restart repo-sync.service
sudo systemctl restart rtcm-server teensy-bridge
```

After updating a script filename (new date stamp), also update the corresponding
`.service` file `ExecStart=` line and reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart <service-name>
```

---

## Known Issues as of 2026-06-14

- **RPWM/LPWM pin swap bug** — confirmed fix identified in steering firmware but
  not yet verified as merged into `teensy_main_20260609.cpp`. Verify before
  field testing tractor02.
- **tractor02 Teensy never flashed** — needs initial firmware upload before
  any hardware testing.
- **led-controller disabled** — known `NameError` bug: `PCA9685` used as type hint
  outside `I2C_AVAILABLE` guard at line 96. Service is disabled on both RPis until fixed.
- **Brenham RTK base raw URL** — confirm correct source file path; currently points
  to tractor_rpi version as placeholder.
