#!/bin/bash
# restart_services.sh - Restart all tractor services

if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root"
    echo "Please run: sudo $0"
    exit 1
fi

echo "Restarting tractor services..."
systemctl restart rtcm-server.service
echo "✓ RTCM Server restarted"
sleep 1
systemctl restart teensy-bridge.service
echo "✓ Teensy Bridge restarted"
sleep 1
systemctl restart led-controller.service
echo "✓ LED Controller restarted"
echo ""
echo "All services restarted successfully!"
