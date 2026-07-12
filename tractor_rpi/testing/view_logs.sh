#!/bin/bash
# view_logs.sh - View logs for a specific service

if [ $# -eq 0 ]; then
    echo "Usage: $0 <service-name>"
    echo ""
    echo "Available services:"
    echo "  rtcm-server"
    echo "  teensy-bridge"
    echo "  led-controller"
    echo "  all           (view all logs interleaved)"
    exit 1
fi

case $1 in
    rtcm-server|teensy-bridge|led-controller)
        echo "Viewing logs for $1 (Ctrl+C to exit)..."
        sudo journalctl -u $1.service -f
        ;;
    all)
        echo "Viewing all tractor service logs (Ctrl+C to exit)..."
        sudo journalctl -u rtcm-server.service -u teensy-bridge.service -u led-controller.service -f
        ;;
    *)
        echo "Unknown service: $1"
        exit 1
        ;;
esac
