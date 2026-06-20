#!/bin/bash
# fix_boot_partition.sh
# Relabels the NVMe boot partition and updates fstab so /boot/firmware
# mounts from NVMe instead of the SD card.

set -e  # exit on any error

echo "=== Current blkid ==="
sudo blkid /dev/mmcblk0p1
sudo blkid /dev/nvme0n1p1

echo ""
echo "=== Current fstab ==="
cat /etc/fstab

echo ""
echo "=== Relabeling NVMe boot partition to 'nvme-boot' ==="
sudo fatlabel /dev/nvme0n1p1 nvme-boot

echo ""
echo "=== Updating fstab ==="
sudo sed -i 's|UUID=F766-26D3|LABEL=nvme-boot|' /etc/fstab
cat /etc/fstab

echo ""
echo "=== Verifying new labels ==="
sudo blkid /dev/nvme0n1p1
sudo blkid /dev/mmcblk0p1

echo ""
echo "=== All done. Rebooting in 5 seconds... ==="
echo "    After reboot, run: findmnt -no SOURCE,TARGET /boot/firmware"
echo "    Expected result:   /dev/nvme0n1p1"
sleep 5
sudo reboot
