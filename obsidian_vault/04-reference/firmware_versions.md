# Firmware Version Log  

Tracks what is actually flashed on each Teensy. The repo contains the source;
this file records which commit was compiled and uploaded to each device.

**Rule:** Update this file immediately after every flash, then commit and push.

---

## Quick Reference — Current Production Files

| Device | Source Folder | Current File |
|--------|--------------|-------------|
| Teensy 4.1 — tractor01/02 | `tractor_teensy/src/` | `teensy_main_20260609.cpp` |
| Teensy 3.2 — RC handheld | `radiocontrol_nrf24radio/src/` | `Rccntrl_rf24_20260609.cpp` |

Older versions archived in `tractor_teensy/src/archive/`

---

## Device Registry

| Device | Source File | Last Flashed | Commit | Flashed From | Notes |
|--------|------------|-------------|--------|-------------|-------|
| Teensy 4.1 — tractor01 | `tractor_teensy/src/teensy_main_20260609.cpp` | unknown | unknown | — | Commit not recorded before this file existed |
| Teensy 4.1 — tractor02 | `tractor_teensy/src/teensy_main_20260609.cpp` | never | — | — | Not yet flashed |
| Teensy 3.2 — RC handheld | `radiocontrol_nrf24radio/src/Rccntrl_rf24_20260609.cpp` | unknown | unknown | — | Commit not recorded before this file existed |

---

## Flash History

| Date | Device | Commit | Notes |
|------|--------|--------|-------|
| 2026-06-14 | — | — | firmware_versions.md created; prior flash history not recorded |

> All entries above this line predate this tracking file.
> From this point forward, record the commit hash at every flash.

---

## Flash Workflow

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

**Step 4 — Update the Device Registry table above, then commit:**
```bash
cd ~/tractor2025
git add firmware_versions.md
git commit -m "firmware_versions: flash <device> at <hash>"
git push origin main
```

---

## RPi Services — Current Production Files

These run as systemd services on the tractor RPis. Updated via `git pull` on boot.
Full details in `tractor_rpi/setup/tractor01_rpi_setup.md` and `tractor02_rpi_setup.md`.

| Service | Current File | Notes |
|---------|-------------|-------|
| rtcm-server | `tractor_rpi/rtcm_server_20260306.py` | GPS pipeline, UDP 6002 |
| teensy-bridge | `tractor_rpi/teensy_serial_bridge_20260310.py` | Serial↔UDP bridge |
| led-controller | `tractor_rpi/led_status_controller.py` | PCA9685 LED tower |
| RTK base Bridgeville | `BridgevilleRTKBase/raspberry-pi/production/rtcm_base_server_20260526.py` | Bridgeville PA |

---

## Known Issues as of 2026-06-14

- **RPWM/LPWM pin swap bug** — confirmed fix identified in steering firmware but
  not yet verified as merged into `teensy_main_20260609.cpp`. Verify before
  field testing tractor02.
- **tractor02 Teensy never flashed** — needs initial firmware upload before
  any hardware testing.
- **tractor01 commit unknown** — was flashed before this tracking file existed;
  update the Device Registry next time tractor01 is reflashed.
