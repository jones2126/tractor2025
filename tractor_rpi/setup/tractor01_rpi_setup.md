# tractor01 RPi 5 — Setup Guide

**Platform:** Raspberry Pi 5, Ubuntu 24.04.03 LTS  
**Hostname:** `tractor` (target: rename to `tractor01` in a future update)  
**User:** `al`  
**Local IP:** `192.168.1.151` | **ZeroTier IP:** `192.168.193.76`

> See also: `tractor02_rpi_setup.md` for the second RPi (different hardware — NVMe, OAK-D).

---

## 0. Flash the SD Card

Use **RPi Imager** and select **Ubuntu 24.04.03 LTS** (not 24.10 or 25.xx).  
Before writing, open the settings and:
- Set hostname, username (`al`), and password
- Enable **WiFi** (enter your SSID and password)
- Enable **SSH**

---

## 1. First Boot

Connect via PuTTY or SSH once the Pi is on the network:

```bash
ssh al@raspberrypi      # or use the IP if hostname doesn't resolve
```

Update all packages:

```bash
sudo apt update && sudo apt upgrade -y
```

Add timestamp to bash history — makes it much easier to reconstruct what was done and when:

```bash
nano ~/.bashrc
# Add this line at the end:
export HISTTIMEFORMAT="%F %T "
source ~/.bashrc
```

Confirm SSH server is installed and running (Ubuntu 24.04 includes it by default, but verify):

```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

---

## 2. ZeroTier — Join robotics_network

ZeroTier allows SSH from any location and connects tractor01 to the rest of the fleet.

**Network ID:** `9f77fc393e0a16f8`

```bash
curl -s https://install.zerotier.com | sudo bash
sudo systemctl enable zerotier-one
sudo systemctl start zerotier-one
sudo zerotier-cli join 9f77fc393e0a16f8
```

Then go to **[my.zerotier.com](https://my.zerotier.com) → Networks → 9f77fc393e0a16f8 → Members**, find the new device, click Edit → check **Authorized**, give it a name, and Save.

```bash
zerotier-cli listnetworks    # confirm assigned IP after authorization
```

---

## 3. Clone the Repo

```bash
sudo apt install git -y
git clone https://github.com/jones2126/tractor2025.git ~/tractor2025
```

Set git pager to avoid interactive pager in terminal sessions:

```bash
git config --global core.pager cat
```

Future updates:

```bash
cd ~/tractor2025
git pull origin main
```

---

## 4. UDEV Rules

Stable `/dev/` symlinks so port assignments don't change on reboot or reconnect.

Create the rules file:

```bash
sudo nano /etc/udev/rules.d/99-tractor-rpi5-devices.rules
```

Paste:

```
# UDEV rules for Tractor RPi 5
# /etc/udev/rules.d/99-tractor-rpi5-devices.rules

# Teensy 4.1 — pinned by USB serial number
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16883800", SYMLINK+="teensy", GROUP="dialout", MODE="0664"

# ZED-F9P for heading — pinned to USB port 4-2
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1546", ENV{ID_MODEL_ID}=="01a9", DEVPATH=="*4-2:1.0*", SYMLINK+="gps-heading", GROUP="dialout", MODE="0664"

# ZED-F9P for base-link (RTCM corrections in) — pinned to USB port 4-1
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1546", ENV{ID_MODEL_ID}=="01a9", DEVPATH=="*4-1:1.0*", SYMLINK+="gps-base-link", GROUP="dialout", MODE="0664"
```

Reload and verify:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/teensy /dev/gps-heading /dev/gps-base-link
```

> **Note:** Port-based GPS rules (`4-1`, `4-2`) depend on which physical USB port is used. If you plug into a different port, the symlink won't appear. Use `udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial"` to confirm after plugging in.

Quick serial test for Teensy:

```bash
python3 -c "import serial; s=serial.Serial('/dev/teensy',460800,timeout=1); print(s.readline())"
```

---

## 5. Repo Sync on Boot

Installs a boot service that checks GitHub on every startup, pulls if behind, and sends a notification via ntfy.sh.

**ntfy topic:** `rpi-tractor-jones2126` (updates automatically if hostname changes)

```bash
bash ~/tractor2025/tractor_rpi/setup/install_repo_sync.sh
```

After running, verify:

```bash
cat /var/log/repo_sync.log
sudo systemctl status repo-sync.service
```

You should receive a ntfy notification on your phone confirming the repo status. From this point on, every boot sends a notification automatically — no SSH session needed.

> **Note:** If the installer pulls new commits, manually restart affected services afterward:
> ```bash
> sudo systemctl restart rtcm-server teensy-bridge led-controller
> ```

---

## 6. Systemd Services

Three services run on boot to operate the robot:

| Service | Script | Purpose |
|---------|--------|---------|
| `rtcm-server` | `rtcm_server_20260306.py` | Receives RTCM GPS correction data, serves it to GPS modules |
| `teensy-bridge` | `teensy_serial_bridge_20260310.py` | Serial bridge between RPi and Teensy 4.1 (starts after rtcm-server) |
| `led-controller` | `led_status_controller.py` | LED tower status display (starts after both above) |

### Install all three services

```bash
cd ~/tractor2025/tractor_rpi
sudo bash install_services.sh
```

The script creates the `.service` files in `/etc/systemd/system/`, enables them for auto-start, and optionally starts them immediately.

### Helper scripts (in `~/tractor2025/tractor_rpi/`)

```bash
bash check_services.sh          # show running/stopped status of all three
sudo bash restart_services.sh   # restart all three in order
bash view_logs.sh all           # tail logs from all three services
bash view_logs.sh teensy-bridge # tail logs from one service
```

### Manual service commands

```bash
# Check status
systemctl status rtcm-server teensy-bridge led-controller

# View live logs
sudo journalctl -u rtcm-server -f
sudo journalctl -u teensy-bridge -f
sudo journalctl -u led-controller -f

# Restart all
sudo systemctl restart rtcm-server teensy-bridge led-controller

# Stop all
sudo systemctl stop rtcm-server teensy-bridge led-controller
```

---

## 7. PlatformIO (Teensy Firmware Upload)

PlatformIO is needed to compile and upload firmware to the Teensy 4.1 from the RPi.

```bash
sudo apt install pipx -y
pipx install platformio
pipx ensurepath
source ~/.bashrc
pio --version    # confirm install
```

Install the PlatformIO udev rules (required for USB upload without sudo):

```bash
sudo wget https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  -O /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Upload firmware to Teensy:

```bash
cd ~/tractor2025/tractor_teensy
pio run --target upload
pio device monitor    # optional: open serial monitor after upload
```

---

## Connection Reference

| Method | Address |
|--------|---------|
| SSH (local) | `ssh al@192.168.1.151` |
| SSH (ZeroTier) | `ssh al@192.168.193.76` |
| PuTTY | `192.168.1.151` |
| Serial bridge | `/dev/teensy` at `460800` baud |

---

## Packages Required (beyond base Ubuntu 24.04)

| Package | Command | Purpose |
|---------|---------|---------|
| openssh-server | `sudo apt install openssh-server -y` | SSH access |
| git | `sudo apt install git -y` | Repo clone |
| python3-serial | `sudo apt install python3-serial -y` | Teensy serial comms |
| curl | `sudo apt install curl -y` | ntfy notifications, ZeroTier install |
| zerotier | `curl -s https://install.zerotier.com \| sudo bash` | Remote VPN access |
| pipx | `sudo apt install pipx -y` | Install PlatformIO |
| platformio | `pipx install platformio` | Compile/upload Teensy firmware |
