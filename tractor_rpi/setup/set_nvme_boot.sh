#!/bin/bash
# Set RPi 5 EEPROM boot order to NVMe first, SD card second.
# Run on tractor02 as: bash ~/tractor2025/tractor_rpi/setup/set_nvme_boot.sh

set -e

echo "Current EEPROM config:"
sudo rpi-eeprom-config
echo ""

# Write config file with NVMe boot order
cat > /tmp/bootconf.txt << 'EOF'
[all]
BOOT_UART=1
POWER_OFF_ON_HALT=0
BOOT_ORDER=0xf416
EOF

echo "Applying new config (NVMe first, SD second)..."
sudo rpi-eeprom-config --apply /tmp/bootconf.txt

echo ""
echo "Verifying:"
sudo rpi-eeprom-config

echo ""
echo "Power cycle the RPi (sudo shutdown now, then unplug/replug)."
echo "After reboot, run: findmnt / -- should show nvme0n1p2"
