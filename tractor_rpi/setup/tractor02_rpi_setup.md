# tractor02 RPi 5 — Setup Guide

**Platform:** Raspberry Pi 5, Ubuntu Server 24.04 LTS  
**Hostname:** `tractor02` | **User:** `al`  
**Local IP:** `192.168.1.213` | **ZeroTier IP:** `192.168.193.48`  
**Script:** `tractor_rpi/setup/tractor02_setup.sh` — run it on the RPi after reading this guide.

---

## Packages Required (beyond base Ubuntu Server 24.04)

| Package | Command | Used for |
|---------|---------|---------|
| wpasupplicant | `sudo apt install wpasupplicant -y` | WiFi (Phase 1) |
| git | `sudo apt install git -y` | Repo clone |
| curl | `sudo apt install curl -y` | ntfy notifications, ZeroTier install |
| python3-serial | `sudo apt install python3-serial -y` | Teensy serial comms |
| python3-pip | `sudo apt install python3-pip -y` | pip (needed before any pip install) |
| wireless-tools | `sudo apt install wireless-tools -y` | WiFi diagnostics (`iwlist wlan0 scan`) |
| nvme-cli | `sudo apt install nvme-cli -y` | NVMe drive info (Phase 4) |
| zerotier | `curl -s https://install.zerotier.com \| sudo bash` | ZeroTier VPN (Phase 2) |
| depthai | `pip install "depthai==2.30.0.0" --break-system-packages` | OAK-D camera (Phase 5) |
| numpy, opencv-python, flask | `pip install numpy opencv-python flask --break-system-packages` | OAK-D dependencies (Phase 5) |

> Update this table whenever a new package is needed during setup.

---

## Status Checklist

| Phase | Task | Status |
|-------|------|--------|
| 1 | WiFi — netplan / wpasupplicant | ✅ Done |
| 2 | ZeroTier — join robotics_network | ✅ Done |
| 3 | BIG7 USB hub + Teensy udev rule | ✅ Done |
| 4 | NVMe SSD hat — detect, clone, set boot order | ✅ Done |
| 5 | OAK-D Lite — udev rule + depthai install + smoke test | ✅ Done |
| 6 | GPS (ZED-X20D) — udev rule + serial test | ✅ Done |
| 7 | Repo sync on boot — install_repo_sync.sh + ntfy | ✅ Done |

---

## 0. First Steps on a Fresh Install

Run these immediately after first boot before anything else:

```bash
# Update all packages
sudo apt update && sudo apt upgrade -y

# Clone the tractor2025 repo
sudo apt install git -y
git clone https://github.com/jones2126/tractor2025.git ~/tractor2025

# Set git pager to avoid interactive pager in terminal sessions
git config --global core.pager cat

# Fix pip PATH so installed scripts are accessible
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
```

---

## 1. WiFi Setup (DONE)

Ubuntu Server 24.04 on the RPi 5 does not have WiFi working out of the box. The fix was to bypass NetworkManager and use netplan + wpasupplicant directly.

### What was tried first (did not work)
- `brcmfmac` driver options via `/etc/modprobe.d/`
- Installing and enabling NetworkManager — it conflicted with netplan

### Working solution

```bash
# Disable NetworkManager
sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager

# Install wpasupplicant
sudo apt update
sudo apt install wpasupplicant -y

# Create netplan WiFi config
sudo nano /etc/netplan/50-wifi.yaml
```

Netplan config template (fill in your SSID and password):

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"
```

```bash
# Lock permissions (netplan requires this)
sudo chmod 600 /etc/netplan/50-wifi.yaml

# Apply and verify
sudo netplan apply
ip addr show wlan0        # should show inet address
ping 8.8.8.8 -c 3
```

> **Note:** If WiFi drops after reboot, check that `50-wifi.yaml` permissions are still 600 and that NetworkManager has not re-enabled itself.

---

## 2. ZeroTier — Join robotics_network (DONE)

ZeroTier allows SSH access from any location and connects tractor02 to the rest of the robotics fleet.

**Network ID:** `9f77fc393e0a16f8`

```bash
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join 9f77fc393e0a16f8
zerotier-cli listnetworks
```

After running: **go to [my.zerotier.com](https://my.zerotier.com) → Networks → 9f77fc393e0a16f8 → Members → authorize the new device.**

Once authorized:
```bash
zerotier-cli listnetworks    # shows assigned ZeroTier IP
```

---

## 3. BIG7 USB Hub + Teensy udev Rule (DONE)

The BIG7 Rev2 is a powered USB hub for the RPi. All USB peripherals (Teensy, GPS, OAK-D) connect through it.

### Verify hub and Teensy detection

```bash
lsusb
dmesg | grep -i usb | tail -20
lsusb | grep "16c0"          # look for Teensy (PJRC VID)
ls /dev/ttyACM*
```

### Teensy udev rule

Teensy 4.1 serial mode: VID=`16c0`, PID=`0483`

```bash
sudo nano /etc/udev/rules.d/99-teensy.rules
```

```
# Teensy 4.1 — creates /dev/teensy symlink
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/teensy
```

Quick serial test:

```bash
python3 -c "import serial; s=serial.Serial('/dev/teensy',460800,timeout=1); print(s.readline())"
```

---

## 4. NVMe SSD Hat (DONE)

NVMe provides faster storage and allows booting without an SD card.

**Result:** tractor02 boots from `nvme0n1p2` (238GB). SD card (`mmcblk0`) remains as fallback.

### Detect the drive

```bash
lsblk    # look for nvme0n1
```

### Clone SD → NVMe

> **Note:** `rpi-clone` fails on NVMe due to partition naming bug — use `dd` instead.

```bash
sudo dd if=/dev/mmcblk0 of=/dev/nvme0n1 bs=4M status=progress conv=fsync
```

Takes ~13 minutes for a 59GB SD card at ~80 MB/s.

### Expand NVMe partition to full drive size

```bash
sudo apt install cloud-guest-utils -y
sudo growpart /dev/nvme0n1 2
sudo e2fsck -f /dev/nvme0n1p2
sudo resize2fs /dev/nvme0n1p2
lsblk /dev/nvme0n1
```

### Set EEPROM boot order to NVMe first

> **Note:** Use `--apply` with a config file, then **power cycle** (not just reboot).

```bash
bash ~/tractor2025/tractor_rpi/setup/set_nvme_boot.sh
sudo shutdown now
# Unplug power, wait 10 seconds, plug back in
```

Verify after boot:
```bash
sudo rpi-eeprom-config    # should show BOOT_ORDER=0xf416
lsblk                     # nvme0n1p1 should show /boot/firmware mountpoint
```

### Fix root partition pointer in cmdline.txt

```bash
sudo blkid /dev/nvme0n1p2          # get PARTUUID
sudo sed -i 's/root=LABEL=writable/root=PARTUUID=a06488b5-02/' /boot/firmware/cmdline.txt
cat /boot/firmware/cmdline.txt     # verify change
sudo reboot
```

Verify after reboot:
```bash
findmnt /    # should show /dev/nvme0n1p2
```

---

## 5. OAK-D Camera (DONE)

See full setup guide: `tractor_rpi/oak-camera/readme.md`

**Quick summary:**
1. Connect OAK-D Lite to BIG7 hub
2. Write udev rule (already done — `99-oak.rules` in place)
3. Install depthai and dependencies (see packages table above)
4. `git pull` on tractor02, then run `python3 ~/tractor2025/tractor_rpi/oak-camera/oak_quick_test.py`

> **Critical:** depthai **must stay at 2.30.0.0** — do not upgrade.

---

## 6. GPS (ZED-X20D) udev Rule (DONE)

Stable `/dev/` names prevent port assignment from changing on reboot or reconnect.

The ArduSimple ZED-X20D on tractor02 uses u-blox native USB (VID=`1546`, PID=`01ab`) and appears
as `/dev/ttyACM*`. The symlink `/dev/gps-heading` is used by `rtcm_server_x20d_20260615.py`.

> **Note:** The PID is `01ab` on this specific unit. The repo file `99-gps-heading.rules` was
> originally written with `01a9` — confirm with `lsusb` before deploying on new hardware.

### Install the rule

```bash
sudo cp ~/tractor2025/tractor_rpi/setup/99-gps-heading.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo reboot
ls -la /dev/gps-heading        # verify symlink exists after reboot
```

### Confirm USB IDs (plug in ZED-X20D first)

```bash
lsusb | grep -i "1546"
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial"
```

Expected on tractor02: `idVendor=="1546"`, `idProduct=="01ab"`. No serial number is presented
by this unit so the rule matches on VID/PID only — this is fine as long as only one u-blox
device is connected.

### Working rule content (as deployed 2026-06-17)

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01ab", SYMLINK+="gps-heading", MODE="0666"
```

### Verify after reboot

```bash
ls -la /dev/gps-heading        # should show -> ttyACM0
```

### Quick serial test

```bash
python3 ~/tractor2025/tractor_rpi/testing/parseDAHEADING.py --port /dev/gps-heading
```

---

## 7. Repo Sync on Boot (DONE)

A boot service checks GitHub on every startup, pulls if behind, and sends a notification via ntfy.sh.

**ntfy topic:** `rpi-tractor02-jones2126`  
**Log file:** `/var/log/repo_sync.log`

### Install

```bash
bash ~/tractor2025/tractor_rpi/setup/install_repo_sync.sh
```

### Verify

```bash
cat /var/log/repo_sync.log
sudo systemctl status repo-sync.service
```

You should receive a ntfy notification on your phone confirming the repo status. From this point on, every boot sends a notification automatically — no SSH session needed.

### Manual run (re-check without rebooting)

```bash
/usr/local/bin/repo_sync_check.sh
```

> **Note:** If a pull occurs, manually restart affected services:
> ```bash
> sudo systemctl restart rtcm-server teensy-bridge led-controller
> ```

---

## 8. Systemd Services

Three services run on boot to operate the robot:

| Service | Script | Purpose |
|---------|--------|---------|
| `rtcm-server` | `rtcm_server_20260617.py` | Receives RTCM GPS correction data, serves it to GPS modules |
| `teensy-bridge` | `teensy_serial_bridge_20260310.py` | Serial bridge between RPi and Teensy 4.1 (starts after rtcm-server) |
| `led-controller` | `led_status_controller.py` | LED tower status display (starts after both above) |

### Install all three services

```bash
cd ~/tractor2025/tractor_rpi
sudo bash install_services.sh
```

### Manual service commands

```bash
systemctl status rtcm-server teensy-bridge led-controller

sudo journalctl -u rtcm-server -f
sudo journalctl -u teensy-bridge -f
sudo journalctl -u led-controller -f

sudo systemctl restart rtcm-server teensy-bridge led-controller
sudo systemctl stop rtcm-server teensy-bridge led-controller
```

---

## Connection Reference

| Method | Address |
|--------|---------|
| SSH (local) | `ssh al@192.168.1.214` |
| SSH (ZeroTier) | `ssh al@192.168.193.48` |
| PuTTY | `192.168.1.214` |

**tractor01 (original RPi 5):** `192.168.1.151` / ZeroTier `192.168.193.76`
