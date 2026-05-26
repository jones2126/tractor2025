# tractor02 RPi 5 — Setup Guide

**Platform:** Raspberry Pi 5, Ubuntu Server 24.04 LTS  
**Hostname:** `tractor02` | **User:** `al`  
**Script:** `tractor_rpi/setup/tractor02_setup.sh` — run it on the RPi after reading this guide.

---

## Packages Required (beyond base Ubuntu Server 24.04)

These are not included in a fresh install and must be added manually:

| Package | Command | Used for |
|---------|---------|---------|
| wpasupplicant | `sudo apt install wpasupplicant -y` | WiFi (Phase 1) |
| git | `sudo apt install git -y` | Repo clone |
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
| 6 | GPS (ZED-F9P) — udev rules + serial test | ⬜ TODO |

---

## 0. First Steps on a Fresh Install

Run these immediately after first boot before anything else:

```bash
# Update all packages
sudo apt update && sudo apt upgrade -y

# Clone the tractor2025 repo
sudo apt install git -y
git clone https://github.com/jones2126/tractor2025.git ~/tractor2025

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

## 2. ZeroTier — Join robotics_network

ZeroTier allows SSH access from any location and connects tractor02 to the rest of the robotics fleet (tractor01, RTK base station, dev laptop).

**Network ID:** `9f77fc393e0a16f8`

```bash
# Install ZeroTier
curl -s https://install.zerotier.com | sudo bash

# Join the robotics network
sudo zerotier-cli join 9f77fc393e0a16f8

# Check status (will show "ACCESS_DENIED" until authorized)
zerotier-cli listnetworks
```

After running: **go to [my.zerotier.com](https://my.zerotier.com) → Networks → 9f77fc393e0a16f8 → Members → authorize the new device.**

Once authorized:
```bash
zerotier-cli listnetworks    # shows assigned ZeroTier IP
```

Update `obsidian_vault/00-project-overview.md` with the new ZeroTier IP for tractor02.

---

## 3. BIG7 USB Hub + Teensy udev Rule

The BIG7 Rev2 is a powered USB hub for the RPi. All USB peripherals (Teensy, GPS, OAK-D) connect through it.

### Verify hub and Teensy detection

```bash
lsusb                        # list all USB devices
dmesg | grep -i usb | tail -20   # recent USB events
lsusb | grep "16c0"          # look for Teensy (PJRC VID)
ls /dev/ttyACM*              # Teensy typically appears here
```

### Create /dev/teensy symlink

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
ls -la /dev/teensy           # verify symlink exists
```

### Quick serial test

```bash
python3 -c "import serial; s=serial.Serial('/dev/teensy',460800,timeout=1); print(s.readline())"
```

---

## 4. NVMe SSD Hat

NVMe provides faster storage and allows booting without an SD card, which is more reliable long-term.

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
sudo apt install cloud-guest-utils -y   # growpart (already in Ubuntu 24.04)
sudo growpart /dev/nvme0n1 2
sudo e2fsck -f /dev/nvme0n1p2           # fix any filesystem errors from live clone
sudo resize2fs /dev/nvme0n1p2
lsblk /dev/nvme0n1                      # nvme0n1p2 should now show full size
```

### Set EEPROM boot order to NVMe first

> **Note:** `rpi-eeprom-config --edit` uses an older firmware binary from the Ubuntu package and reverts the config. Use `--apply` with a config file instead, then **power cycle** (not just reboot).

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

After setting boot order, `/boot/firmware` loads from NVMe but root `/` still mounts from SD card because both have `LABEL=writable`. Fix by pointing cmdline.txt to the NVMe PARTUUID:

```bash
sudo blkid /dev/nvme0n1p2          # get PARTUUID (e.g. a06488b5-02)
sudo sed -i 's/root=LABEL=writable/root=PARTUUID=a06488b5-02/' /boot/firmware/cmdline.txt
cat /boot/firmware/cmdline.txt     # verify change
sudo reboot
```

Verify after reboot:
```bash
findmnt /    # should show /dev/nvme0n1p2
```

---

## 5. OAK-D Camera

See full setup guide: `tractor_rpi/oak-camera/readme.md`

**Quick summary:**
1. Connect OAK-D Lite to BIG7 hub
2. Write udev rule (already done — `99-oak.rules` in place)
3. Install depthai and dependencies (see packages table above)
4. `git pull` on tractor02, then run `python3 ~/tractor2025/tractor_rpi/oak-camera/oak_quick_test.py`

---

## 6. GPS (ZED-F9P) udev Rules

Stable `/dev/` names prevent port assignment from changing on reboot or reconnect.

### Identify GPS ports

```bash
lsusb                        # find VID:PID for GPS adapters
ls /dev/ttyUSB* /dev/ttyACM*

# Get serial number for each GPS (plug in one at a time)
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
```

ArduSimple ZED-F9P boards typically use a CP210x USB adapter (VID=`10c4`, PID=`ea60`). Use the `ATTRS{serial}` value to distinguish rover from base.

### Write GPS udev rule

```bash
sudo nano /etc/udev/rules.d/99-gps.rules
```

```
# GPS rover — /dev/gps0
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ROVER_SERIAL_HERE", SYMLINK+="gps0", MODE="0666"

# GPS base / correction input — /dev/gps1
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="BASE_SERIAL_HERE", SYMLINK+="gps1", MODE="0666"
```

Replace `ROVER_SERIAL_HERE` and `BASE_SERIAL_HERE` with the values from `udevadm info`.

### OAK-D Camera (Luxonis)

OAK-D VID=`03e7` — needs a udev rule for non-root USB access.

```bash
lsusb | grep "03e7"          # verify OAK-D detected

sudo nano /etc/udev/rules.d/99-oak.rules
```

```
# Luxonis OAK-D — grants non-root USB access
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Test OAK-D (depthai **must stay at 2.30.0.0** — do not upgrade):

```bash
python3 -c "import depthai as dai; print(dai.__version__)"
python3 -c "import depthai as dai; d=dai.Device(); print('OAK-D connected:', d.getMxId())"
```

---

## Connection Reference

| Method | Address |
|--------|---------|
| SSH (local) | `ssh al@192.168.1.214` |
| SSH (ZeroTier) | `ssh al@192.168.193.48` |
| PuTTY | Use local IP above |

**tractor01 (original RPi 5):** `192.168.1.151` / ZeroTier `192.168.193.76`
