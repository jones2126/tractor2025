#!/bin/bash
# tractor02_setup.sh
# RPi 5 initial setup for tractor02 (tractor robot second RPi).
# Run as user 'al' — sudo prompted as needed.
# See obsidian_vault/04-reference/tractor02-rpi-setup.md for full notes.
#
# Usage: bash tractor02_setup.sh
#
# Phases:
#   1 - WiFi         (DONE — netplan/wpasupplicant, already configured)
#   2 - ZeroTier     (join robotics_network 9f77fc393e0a16f8)
#   3 - BIG7 USB hub + Teensy udev rule
#   4 - NVMe SSD hat
#   5 - GPS (ZED-F9P) + OAK-D udev rules

set -e

echo "=== tractor02 RPi 5 Setup ==="
echo ""

# -------------------------------------------------------------------
# PHASE 1: WiFi — ALREADY DONE
# Kept here for reference only. Do NOT re-run; it will overwrite working config.
# -------------------------------------------------------------------
phase1_wifi_reference() {
    echo "[PHASE 1] WiFi — reference only, already configured."
    echo ""
    echo "  What was done (Ubuntu Server 24.04 had no WiFi out of the box):"
    echo "  - brcmfmac driver options attempted first (did not resolve issue)"
    echo "  - NetworkManager installed but conflicted with netplan — disabled"
    echo "  - wpasupplicant installed as the backend"
    echo "  - Netplan config written to /etc/netplan/50-wifi.yaml"
    echo "  - Permissions locked: chmod 600 /etc/netplan/50-wifi.yaml"
    echo "  - sudo netplan apply — WiFi came up"
    echo ""
    echo "  Current WiFi status:"
    ip addr show wlan0 | grep "inet " || echo "  wlan0 has no IP — check config"
    ping -c 2 8.8.8.8 && echo "  Internet reachable." || echo "  No internet."
}

# -------------------------------------------------------------------
# PHASE 2: ZeroTier — join robotics_network
# -------------------------------------------------------------------
phase2_zerotier() {
    echo "[PHASE 2] ZeroTier"
    echo ""

    if command -v zerotier-cli &> /dev/null; then
        echo "  ZeroTier already installed."
    else
        echo "  Installing ZeroTier..."
        curl -s https://install.zerotier.com | sudo bash
        echo "  ZeroTier installed."
    fi

    echo ""
    echo "  Joining robotics_network (9f77fc393e0a16f8)..."
    sudo zerotier-cli join 9f77fc393e0a16f8

    echo ""
    echo "  IMPORTANT: Go to my.zerotier.com and authorize this device."
    echo "  Once authorized, your ZeroTier IP will appear below:"
    echo ""
    sleep 5
    zerotier-cli listnetworks

    echo ""
    echo "  To check ZeroTier IP after authorization:"
    echo "    zerotier-cli listnetworks"
    echo "    ip addr show ztXXXXXXXX"
}

# -------------------------------------------------------------------
# PHASE 3: BIG7 USB Hub + Teensy udev rule
# -------------------------------------------------------------------
phase3_usb_hub() {
    echo "[PHASE 3] BIG7 USB Hub + Teensy udev rule"
    echo ""

    echo "  Listing USB devices (run with hub plugged in)..."
    lsusb
    echo ""

    echo "  Recent USB events from dmesg:"
    dmesg | grep -i usb | tail -20
    echo ""

    echo "  Looking for Teensy device (PJRC VID 16c0)..."
    lsusb | grep -i "16c0" || echo "  Teensy not found — is it connected through the hub?"
    echo ""

    # Teensy 4.1 udev rule — creates /dev/teensy symlink
    UDEV_FILE="/etc/udev/rules.d/99-teensy.rules"
    if [ -f "$UDEV_FILE" ]; then
        echo "  Teensy udev rule already exists at $UDEV_FILE:"
        cat "$UDEV_FILE"
    else
        echo "  Writing Teensy udev rule to $UDEV_FILE ..."
        # PJRC Teensy 4.1 serial mode: VID=16c0, PID=0483
        sudo tee "$UDEV_FILE" > /dev/null <<'EOF'
# Teensy 4.1 — creates /dev/teensy symlink
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"
EOF
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "  Udev rule written and reloaded."
    fi

    echo ""
    echo "  Checking for /dev/teensy ..."
    ls -la /dev/teensy 2>/dev/null || echo "  /dev/teensy not present — reconnect Teensy or check hub."

    echo ""
    echo "  Quick serial test (requires Teensy running firmware at 460800 baud):"
    echo "    python3 -c \"import serial; s=serial.Serial('/dev/teensy',460800,timeout=1); print(s.readline())\""
}

# -------------------------------------------------------------------
# PHASE 4: NVMe SSD Hat
# -------------------------------------------------------------------
phase4_nvme() {
    echo "[PHASE 4] NVMe SSD Hat"
    echo ""

    echo "  Checking for NVMe drive..."
    lsblk | grep -E "nvme|NAME" || echo "  No NVMe device found — is the HAT seated?"
    echo ""

    if ! command -v nvme &> /dev/null; then
        echo "  Installing nvme-cli..."
        sudo apt install -y nvme-cli
    fi

    echo "  NVMe drive details:"
    sudo nvme list 2>/dev/null || echo "  nvme list failed — drive may not be detected."
    echo ""

    echo "  Current EEPROM boot order:"
    sudo rpi-eeprom-config | grep BOOT_ORDER || echo "  Could not read EEPROM config."
    echo ""

    echo "  --- To enable NVMe boot (run after cloning SD to NVMe) ---"
    echo "    sudo rpi-eeprom-config --edit"
    echo "    Set: BOOT_ORDER=0xf416  (NVMe first, then SD, then USB)"
    echo "    Then: sudo reboot"
    echo ""

    echo "  --- To clone SD to NVMe using rpi-clone ---"
    echo "    sudo apt install -y git"
    echo "    git clone https://github.com/billw2/rpi-clone.git /tmp/rpi-clone"
    echo "    sudo cp /tmp/rpi-clone/rpi-clone /usr/local/sbin/"
    echo "    sudo rpi-clone nvme0n1"
    echo ""

    echo "  --- Quick write benchmark (after NVMe is mounted) ---"
    echo "    sudo dd if=/dev/zero of=/mnt/nvme/test bs=1M count=512 oflag=dsync && rm /mnt/nvme/test"
}

# -------------------------------------------------------------------
# PHASE 5: GPS (ZED-F9P) + OAK-D udev rules
# -------------------------------------------------------------------
phase5_udev_gps_oak() {
    echo "[PHASE 5] GPS (ZED-F9P) + OAK-D udev rules"
    echo ""

    echo "  Current USB devices:"
    lsusb
    echo ""

    echo "  === GPS: ArduSimple ZED-F9P ==="
    echo "  Looking for known GPS USB adapters (u-blox 1546, CP210x 10c4, FTDI 0403)..."
    lsusb | grep -iE "1546|10c4|0403" || echo "  No GPS USB device found — is it plugged in?"
    echo ""
    echo "  To find port and attributes for udev rule:"
    echo "    ls /dev/ttyUSB* /dev/ttyACM*"
    echo "    udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'"
    echo ""

    GPS_UDEV="/etc/udev/rules.d/99-gps.rules"
    if [ -f "$GPS_UDEV" ]; then
        echo "  GPS udev rule already exists at $GPS_UDEV:"
        cat "$GPS_UDEV"
    else
        echo "  GPS udev rule not yet written."
        echo "  After confirming VID/PID and serial numbers with udevadm, create:"
        echo "    sudo nano $GPS_UDEV"
        echo ""
        echo "  Template (adjust ATTRS{serial} per device):"
        cat <<'EOF'
# GPS rover — /dev/gps0
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ROVER_SERIAL_HERE", SYMLINK+="gps0", MODE="0666"
# GPS base / correction input — /dev/gps1
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="BASE_SERIAL_HERE", SYMLINK+="gps1", MODE="0666"
EOF
    fi
    echo ""

    echo "  === OAK-D Camera (Luxonis) ==="
    echo "  Looking for OAK-D (VID 03e7)..."
    lsusb | grep "03e7" || echo "  OAK-D not found — is it plugged in?"

    OAK_UDEV="/etc/udev/rules.d/99-oak.rules"
    if [ -f "$OAK_UDEV" ]; then
        echo "  OAK-D udev rule already exists at $OAK_UDEV:"
        cat "$OAK_UDEV"
    else
        echo "  Writing OAK-D udev rule to $OAK_UDEV ..."
        sudo tee "$OAK_UDEV" > /dev/null <<'EOF'
# Luxonis OAK-D — grants non-root USB access
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "  OAK-D udev rule written and reloaded."
    fi

    echo ""
    echo "  To test OAK-D (depthai must be 2.30.0.0 — do NOT upgrade):"
    echo "    python3 -c \"import depthai as dai; print(dai.__version__)\""
    echo "    python3 -c \"import depthai as dai; d=dai.Device(); print('OAK-D connected:', d.getMxId())\""
}

# -------------------------------------------------------------------
# Main menu
# -------------------------------------------------------------------
echo "Select a phase to run:"
echo "  1 — WiFi status check (reference, already done)"
echo "  2 — ZeroTier (join robotics_network)"
echo "  3 — BIG7 USB hub + Teensy udev"
echo "  4 — NVMe SSD hat"
echo "  5 — GPS + OAK-D udev rules"
echo "  all — run all phases in order"
echo ""
read -rp "Enter phase number (or 'all'): " PHASE

case "$PHASE" in
    1) phase1_wifi_reference ;;
    2) phase2_zerotier ;;
    3) phase3_usb_hub ;;
    4) phase4_nvme ;;
    5) phase5_udev_gps_oak ;;
    all)
        phase1_wifi_reference; echo "---"
        phase2_zerotier; echo "---"
        phase3_usb_hub; echo "---"
        phase4_nvme; echo "---"
        phase5_udev_gps_oak
        ;;
    *) echo "Usage: bash tractor02_setup.sh  (then choose a phase)" ;;
esac

echo ""
echo "=== Done ==="
