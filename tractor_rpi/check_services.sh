#!/bin/bash
# check_services.sh - Check status of all tractor services
# to run from any directory: /home/al/tractor2025/tractor_rpi/check_services.sh

echo "========================================="
echo "Tractor Robot Service Status"
echo "========================================="
echo ""

services=("rtcm-server" "teensy-bridge" "led-controller")

for service in "${services[@]}"; do
    echo "[$service]"
    if systemctl is-active --quiet $service.service; then
        echo "  Status: RUNNING ✓"
    else
        echo "  Status: STOPPED ✗"
    fi
    
    if systemctl is-enabled --quiet $service.service; then
        echo "  Boot: ENABLED ✓"
    else
        echo "  Boot: DISABLED ✗"
    fi
    echo ""
done

echo "========================================="
echo "Quick Commands:"
echo "  View logs:    sudo journalctl -u <service-name> -f"
echo "  Restart all:  sudo systemctl restart rtcm-server teensy-bridge led-controller"
echo "  Stop all:     sudo systemctl stop rtcm-server teensy-bridge led-controller"
echo "========================================="
