# tractor02 RPi 5 — Setup Guide

**Platform:** Raspberry Pi 5, Ubuntu Server 24.04 LTS  
**Hostname:** `tractor02` | **User:** `al`  
**Local IP:** `192.168.1.213` | **ZeroTier IP:** `192.168.193.48`  
**Script:** `tractor_rpi/setup/tractor02_setup.sh` — run it on the RPi after reading this guide.

---

## Packages Required (beyond base Ubuntu Server 24.04)

| Package | Command | Used for |
| :---- | :---- | :---- |
| wpasupplicant | `sudo apt install wpasupplicant -y` | WiFi (Phase 1\) |
| git | `sudo apt install git -y` | Repo clone |
| curl | `sudo apt install curl -y` | ntfy notifications, ZeroTier install |
| python3-serial | `sudo apt install python3-serial -y` | Teensy serial comms |
| python3-pip | `sudo apt install python3-pip -y` | pip (needed before any pip install) |
| wireless-tools | `sudo apt install wireless-tools -y` | WiFi diagnostics (`iwlist wlan0 scan`) |
| nvme-cli | `sudo apt install nvme-cli -y` | NVMe drive info (Phase 4\) |
| zerotier | `curl -s https://install.zerotier.com | sudo bash` | ZeroTier VPN (Phase 2\) |
| depthai | `pip install "depthai==2.30.0.0" --break-system-packages` | OAK-D camera (Phase 5\) |
| numpy, opencv-python, flask | `pip install numpy opencv-python flask --break-system-packages` | OAK-D dependencies (Phase 5\) |
| platformio | `pip install platformio --break-system-packages` | platformio |

Update this table whenever a new package is needed during setup.

---

## Status Checklist

| Phase | Task | Status |
| :---- | :---- | :---- |
| 1 | WiFi — netplan / wpasupplicant | ✅ Done |
| 2 | ZeroTier — join robotics\_network | ✅ Done |
| 3 | BIG7 USB hub \+ Teensy udev rule | ✅ Done |
| 4 | NVMe SSD hat — detect, clone, set boot order, fix boot partition | ✅ Done |
| 5 | OAK-D Lite — udev rule \+ depthai install \+ smoke test | ✅ Done |
| 6 | GPS (ZED-X20D) — udev rule \+ serial test | ✅ Done |
| 7 | Repo sync on boot — install\_repo\_sync.sh \+ ntfy | ✅ Done |
| 8 | Disable cloud-init | ✅ Done |
| 9 | PlatformIO CLI — install + verify | ☐ Pending |

---

## 0\. First Steps on a Fresh Install

Run these immediately after first boot before anything else:

\# Update all packages

sudo apt update && sudo apt upgrade \-y

\# Clone the tractor2025 repo

sudo apt install git \-y

git clone https://github.com/jones2126/tractor2025.git \~/tractor2025

\# Set git pager to avoid interactive pager in terminal sessions

git config \--global core.pager cat

\# Fix pip PATH so installed scripts are accessible

echo 'export PATH="$HOME/.local/bin:$PATH"' \>\> \~/.bashrc && source \~/.bashrc

---

## 1\. WiFi Setup (DONE)

Ubuntu Server 24.04 on the RPi 5 does not have WiFi working out of the box. The fix was to bypass NetworkManager and use netplan \+ wpasupplicant directly.

### What was tried first (did not work)

- `brcmfmac` driver options via `/etc/modprobe.d/`  
- Installing and enabling NetworkManager — it conflicted with netplan

### Working solution

\# Disable NetworkManager

sudo systemctl stop NetworkManager

sudo systemctl disable NetworkManager

\# Install wpasupplicant

sudo apt update

sudo apt install wpasupplicant \-y

\# Create netplan WiFi config

sudo nano /etc/netplan/50-wifi.yaml

Netplan config template (fill in your SSID and password):

network:

  version: 2

  wifis:

    wlan0:

      dhcp4: true

      access-points:

        "YOUR\_SSID":

          password: "YOUR\_PASSWORD"

\# Lock permissions (netplan requires this)

sudo chmod 600 /etc/netplan/50-wifi.yaml

\# Apply and verify

sudo netplan apply

ip addr show wlan0        \# should show inet address

ping 8.8.8.8 \-c 3

**Note:** If WiFi drops after reboot, check that `50-wifi.yaml` permissions are still 600 and that NetworkManager has not re-enabled itself.

---

Suppress brcmfmac P2P warning (DONE)

Nearby Wi-Fi Direct devices (e.g. HP printers) cause the brcmfmac driver to log:
ieee80211 phy0: brcmf_p2p_send_action_frame: Unknown Frame: category 0xa, action 0x8
This is harmless but noisy. Suppress it by disabling P2P mode on the driver:

bashecho 'options brcmfmac p2pon=0' | sudo tee /etc/modprobe.d/brcmfmac.conf

sudo reboot

# Confirm the p2p line is absent:
sudo dmesg | grep -i brcmf

---

## 2\. ZeroTier — Join robotics\_network (DONE)

ZeroTier allows SSH access from any location and connects tractor02 to the rest of the robotics fleet.

**Network ID:** `9f77fc393e0a16f8`

curl \-s https://install.zerotier.com | sudo bash

sudo zerotier-cli join 9f77fc393e0a16f8

zerotier-cli listnetworks

After running: **go to [my.zerotier.com](https://my.zerotier.com) → Networks → 9f77fc393e0a16f8 → Members → authorize the new device.**

Once authorized:

zerotier-cli listnetworks    \# shows assigned ZeroTier IP

---

## 3\. BIG7 USB Hub \+ Teensy udev Rule (DONE)

The BIG7 Rev2 is a powered USB hub for the RPi. All USB peripherals (Teensy, GPS, OAK-D) connect through it.

### Verify hub and Teensy detection

lsusb

dmesg | grep \-i usb | tail \-20

lsusb | grep "16c0"          \# look for Teensy (PJRC VID)

ls /dev/ttyACM\*

### Teensy udev rule

Teensy 4.1 serial mode: VID=`16c0`, PID=`0483`

sudo nano /etc/udev/rules.d/99-teensy.rules

\# Teensy 4.1 — creates /dev/teensy symlink

SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"

sudo udevadm control \--reload-rules

sudo udevadm trigger

ls \-la /dev/teensy

Quick serial test:

python3 \-c "import serial; s=serial.Serial('/dev/teensy',460800,timeout=1); print(s.readline())"

---

## 4\. NVMe SSD Hat (DONE)

NVMe provides faster storage and allows booting without an SD card.

**Final verified state (2026-06-20):**

- Root filesystem (`/`) → `nvme0n1p2` (238GB) ✅  
- Boot partition (`/boot/firmware`) → `nvme0n1p1` ✅  
- SD card is no longer needed and can be removed

### Detect the drive

lsblk    \# look for nvme0n1

### Clone SD → NVMe

**Note:** `rpi-clone` fails on NVMe due to partition naming bug — use `dd` instead.

sudo dd if=/dev/mmcblk0 of=/dev/nvme0n1 bs=4M status=progress conv=fsync

Takes \~13 minutes for a 59GB SD card at \~80 MB/s.

### Expand NVMe partition to full drive size

sudo apt install cloud-guest-utils \-y

sudo growpart /dev/nvme0n1 2

sudo e2fsck \-f /dev/nvme0n1p2

sudo resize2fs /dev/nvme0n1p2

lsblk /dev/nvme0n1

### Set EEPROM boot order to NVMe first

**Note:** Use `--apply` with a config file, then **power cycle** (not just reboot).

bash \~/tractor2025/tractor\_rpi/setup/set\_nvme\_boot.sh

sudo shutdown now

\# Unplug power, wait 10 seconds, plug back in

Verify after boot:

sudo rpi-eeprom-config    \# should show BOOT\_ORDER=0xf416

lsblk                     \# nvme0n1p1 should show /boot/firmware mountpoint

### Fix root partition pointer in cmdline.txt

sudo blkid /dev/nvme0n1p2          \# get PARTUUID

sudo sed \-i 's/root=LABEL=writable/root=PARTUUID=a06488b5-02/' /boot/firmware/cmdline.txt

cat /boot/firmware/cmdline.txt     \# verify change

sudo reboot

Verify after reboot:

findmnt /    \# should show /dev/nvme0n1p2

### Fix /boot/firmware to mount from NVMe (not SD card)

After the `dd` clone, both the SD card and NVMe boot partitions have identical UUIDs and labels (`system-boot`, UUID `F766-26D3`). Linux grabs whichever it finds first — usually the SD card — so `/boot/firmware` ends up on the SD card even though root is on NVMe.

The fix is to give the NVMe boot partition a unique label and update fstab to reference it. A script is provided: `tractor_rpi/setup/fix_boot_partition.sh`

bash \~/tractor2025/tractor\_rpi/setup/fix\_boot\_partition.sh

What the script does:

1. Relabels `nvme0n1p1` from `system-boot` to `nvme-boot`  
2. Updates `/etc/fstab` to mount `/boot/firmware` by `LABEL=nvme-boot`  
3. Reboots automatically

After reboot, verify:

findmnt \-no SOURCE,TARGET /               \# should show /dev/nvme0n1p2

findmnt \-no SOURCE,TARGET /boot/firmware  \# should show /dev/nvme0n1p1

Both on NVMe \= success. SD card can now be removed.

---

## 5\. OAK-D Camera (DONE)

See full setup guide: `tractor_rpi/oak-camera/readme.md`

**Quick summary:**

1. Connect OAK-D Lite to BIG7 hub  
2. Write udev rule (already done — `99-oak.rules` in place)  
3. Install depthai and dependencies (see packages table above)  
4. `git pull` on tractor02, then run `python3 ~/tractor2025/tractor_rpi/oak-camera/oak_quick_test.py`

**Critical:** depthai **must stay at 2.30.0.0** — do not upgrade.

---

## 6\. GPS (ZED-X20D) udev Rule (DONE)

Stable `/dev/` names prevent port assignment from changing on reboot or reconnect.

The ArduSimple ZED-X20D on tractor02 uses u-blox native USB (VID=`1546`, PID=`01ab`) and appears as `/dev/ttyACM*`. The symlink `/dev/gps-heading` is used by `rtcm_server_x20d_20260615.py`.

**Note:** The PID is `01ab` on this specific unit. The repo file `99-gps-heading.rules` was originally written with `01a9` — confirm with `lsusb` before deploying on new hardware.

### Install the rule

sudo cp \~/tractor2025/tractor\_rpi/setup/99-gps-heading.rules /etc/udev/rules.d/

sudo udevadm control \--reload-rules

sudo reboot

ls \-la /dev/gps-heading        \# verify symlink exists after reboot

### Confirm USB IDs (plug in ZED-X20D first)

lsusb | grep \-i "1546"

udevadm info \-a \-n /dev/ttyACM0 | grep \-E "idVendor|idProduct|serial"

Expected on tractor02: `idVendor=="1546"`, `idProduct=="01ab"`. No serial number is presented by this unit so the rule matches on VID/PID only — this is fine as long as only one u-blox device is connected.

### Working rule content (as deployed 2026-06-17)

SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01ab", SYMLINK+="gps-heading", MODE="0666"

### Verify after reboot

ls \-la /dev/gps-heading        \# should show \-\> ttyACM0

### Quick serial test

python3 \~/tractor2025/tractor\_rpi/testing/parseDAHEADING.py \--port /dev/gps-heading

---

## 7\. Repo Sync on Boot (DONE)

A boot service checks GitHub on every startup, pulls if behind, and sends a notification via ntfy.sh.

**ntfy topic:** `rpi-tractor02-jones2126`  
**Log file:** `/var/log/repo_sync.log`

### Install

bash \~/tractor2025/tractor\_rpi/setup/install\_repo\_sync.sh

### Verify

cat /var/log/repo\_sync.log

sudo systemctl status repo-sync.service

You should receive a ntfy notification on your phone confirming the repo status. From this point on, every boot sends a notification automatically — no SSH session needed.

### Manual run (re-check without rebooting)

/usr/local/bin/repo\_sync\_check.sh

**Note:** If a pull occurs, manually restart affected services:

sudo systemctl restart rtcm-server teensy-bridge led-controller

---

### **8\. Disable cloud-init (DONE)**

Ubuntu Server uses cloud-init for first-boot provisioning (user accounts, SSH keys, hostname,  
 etc.). On tractor02 that work is already done and cloud-init has nothing useful left to do.

Additionally, the Raspberry Pi Imager injects a non-standard `enable_ssh` key into the  
 cloud-init user-data. Ubuntu's cloud-init schema validator does not recognize this key and  
 logs a warning on every boot:

cloud-config failed schema validation\!  
enable\_ssh: Additional properties are not allowed ('enable\_ssh' was unexpected)

Since tractor02 is a stable, configured machine running Ubuntu (not Raspberry Pi OS), and  
 images are pushed to it rather than created from it, the correct fix is to disable cloud-init  
 entirely.

**Note:** `sudo touch /etc/cloud/cloud-init.disabled` alone does not survive a reboot on  
 Ubuntu — cloud-init or a boot process removes the file. Masking the systemd units is  
 required to make the change permanent.

bash  
sudo touch /etc/cloud/cloud-init.disabled  
sudo systemctl disable cloud-init cloud-init-local cloud-config cloud-final  
sudo systemctl mask cloud-init cloud-init-local cloud-config cloud-final  
sudo reboot

\# After reboot, confirm:  
sudo cloud-init status    \# should show: status: disabled
---
## 9. PlatformIO CLI (serial monitor only)

tractor02 is not used to compile or flash firmware — that is done on the laptop via
VSCode + PlatformIO. However, the `pio device monitor` command is useful for
monitoring raw Teensy serial output during bench testing, as an alternative to
`python3 -c "import serial; ..."`.

**Note:** Only install PlatformIO CLI, not the full IDE. The `--user` install keeps
it out of system Python.

### Install

```bash
pip install platformio --break-system-packages
```

Verify PATH is set (should already be from Phase 0 `.bashrc` update):

```bash
source ~/.bashrc
pio --version
```

### Usage

```bash
# Monitor Teensy serial output at firmware baud rate
pio device monitor --port /dev/teensy --baud 460800
```

**Important:** Only one process can own `/dev/teensy` at a time.
Stop `teensy-bridge` before using the monitor:

```bash
sudo systemctl stop teensy-bridge
pio device monitor --port /dev/teensy --baud 460800
# Ctrl+C to exit, then:
sudo systemctl start teensy-bridge
```

**Note on baud rate:** The firmware baud rate is **460800** (set in
`teensy_main_20260609.cpp`). Using 115200 will produce garbled output.
---

## 10\. Systemd Services

Three services run on boot to operate the robot:

| Service | Script | Purpose |
| :---- | :---- | :---- |
| `rtcm-server` | `rtcm_server_20260617.py` | Receives RTCM GPS correction data, serves it to GPS modules |
| `teensy-bridge` | `teensy_serial_bridge_20260310.py` | Serial bridge between RPi and Teensy 4.1 (starts after rtcm-server) |
| `led-controller` | `led_status_controller.py` | LED tower status display (starts after both above) |

### Install all three services

cd \~/tractor2025/tractor\_rpi

sudo bash install\_services.sh

### Manual service commands

systemctl status rtcm-server teensy-bridge led-controller

sudo journalctl \-u rtcm-server \-f

sudo journalctl \-u teensy-bridge \-f

sudo journalctl \-u led-controller \-f

sudo systemctl restart rtcm-server teensy-bridge led-controller

sudo systemctl stop rtcm-server teensy-bridge led-controller

---

## Hardware Notes

### Power Supply

Use the official Raspberry Pi 5 27W USB-C power supply (5V/5A). A standard RPi 4 supply (5V/3A) will trigger an undervoltage warning and restrict power to peripherals. Running underpowered also causes higher idle temperatures.

### Temperature

Normal idle temperature with adequate cooling: **40–50°C**.  
Throttle threshold: **85°C**. Hard limit: \~90°C.

If idle temperature is above 55°C, check:

- Power supply (underpowered boards run hotter)  
- Heatsink seating on the SoC  
- Airflow / enclosure ventilation

tractor02 was observed at **62.8°C idle on 2026-06-20** with an underpowered supply. Upgrading to the correct 5A supply is recommended before field deployment.

---

## Connection Reference

| Method | Address |
| :---- | :---- |
| SSH (local) | `ssh al@192.168.1.214` |
| SSH (ZeroTier) | `ssh al@192.168.193.48` |
| PuTTY | `192.168.1.214` |

**tractor01 (original RPi 5):** `192.168.1.151` / ZeroTier `192.168.193.76`  
