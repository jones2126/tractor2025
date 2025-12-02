#!/bin/bash
# install_services.sh
# Installation script for tractor robot systemd services
# 
# This script installs three systemd services:
# 1. rtcm-server.service - GPS/RTK correction data server
# 2. teensy-bridge.service - Teensy serial communication bridge
# 3. led-controller.service - LED status tower controller

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="/home/al/tractor2025/tractor_rpi"
USER="al"
PYTHON_BIN="/usr/bin/python3"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Tractor Robot Service Installation${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}Error: This script must be run as root${NC}"
    echo "Please run: sudo $0"
    exit 1
fi

# Verify script directory exists
if [ ! -d "$SCRIPT_DIR" ]; then
    echo -e "${RED}Error: Script directory does not exist: $SCRIPT_DIR${NC}"
    exit 1
fi

# Verify Python scripts exist
SCRIPTS=("rtcm_server.py" "teensy_serial_bridge.py" "led_status_controller.py")
for script in "${SCRIPTS[@]}"; do
    if [ ! -f "$SCRIPT_DIR/$script" ]; then
        echo -e "${RED}Error: Script not found: $SCRIPT_DIR/$script${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓${NC} Script directory verified: $SCRIPT_DIR"
echo -e "${GREEN}✓${NC} All Python scripts found"
echo ""

# Make scripts executable
echo "Making scripts executable..."
chmod +x "$SCRIPT_DIR"/*.py
chown -R $USER:$USER "$SCRIPT_DIR"
echo -e "${GREEN}✓${NC} Scripts are now executable"
echo ""

# Create service files
echo "Creating systemd service files..."

# 1. RTCM Server Service
cat > /etc/systemd/system/rtcm-server.service << EOF
[Unit]
Description=RTCM Server for GPS/RTK Corrections
Documentation=https://github.com/jones2126/tractor2025
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$PYTHON_BIN $SCRIPT_DIR/rtcm_server.py
Restart=always
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=rtcm-server

# Resource limits (optional)
# MemoryLimit=256M
# CPUQuota=50%

[Install]
WantedBy=multi-user.target
EOF

echo -e "${GREEN}✓${NC} Created rtcm-server.service"

# 2. Teensy Bridge Service
cat > /etc/systemd/system/teensy-bridge.service << EOF
[Unit]
Description=Teensy Serial Bridge
Documentation=https://github.com/jones2126/tractor2025
After=network.target rtcm-server.service
Wants=rtcm-server.service

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$PYTHON_BIN $SCRIPT_DIR/teensy_serial_bridge.py
Restart=always
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=teensy-bridge

# Resource limits (optional)
# MemoryLimit=256M
# CPUQuota=50%

[Install]
WantedBy=multi-user.target
EOF

echo -e "${GREEN}✓${NC} Created teensy-bridge.service"

# 3. LED Controller Service
cat > /etc/systemd/system/led-controller.service << EOF
[Unit]
Description=LED Status Controller
Documentation=https://github.com/jones2126/tractor2025
After=network.target teensy-bridge.service rtcm-server.service
Wants=teensy-bridge.service rtcm-server.service

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$PYTHON_BIN $SCRIPT_DIR/led_status_controller.py
Restart=always
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=led-controller

# Resource limits (optional)
# MemoryLimit=256M
# CPUQuota=50%

[Install]
WantedBy=multi-user.target
EOF

echo -e "${GREEN}✓${NC} Created led-controller.service"
echo ""

# Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload
echo -e "${GREEN}✓${NC} Systemd daemon reloaded"
echo ""

# Enable services
echo "Enabling services to start on boot..."
systemctl enable rtcm-server.service
systemctl enable teensy-bridge.service
systemctl enable led-controller.service
echo -e "${GREEN}✓${NC} Services enabled for auto-start on boot"
echo ""

# Ask user if they want to start services now
read -p "Do you want to start the services now? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Starting services..."
    systemctl start rtcm-server.service
    sleep 2
    systemctl start teensy-bridge.service
    sleep 2
    systemctl start led-controller.service
    echo -e "${GREEN}✓${NC} Services started"
    echo ""
    
    # Show status
    echo "Service Status:"
    echo "==============="
    systemctl status rtcm-server.service --no-pager -l | head -n 5
    echo ""
    systemctl status teensy-bridge.service --no-pager -l | head -n 5
    echo ""
    systemctl status led-controller.service --no-pager -l | head -n 5
    echo ""
fi

# Create helper scripts
echo "Creating helper scripts..."

# Status check script
cat > "$SCRIPT_DIR/check_services.sh" << 'EOFHELPER'
#!/bin/bash
# check_services.sh - Check status of all tractor services

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
EOFHELPER

chmod +x "$SCRIPT_DIR/check_services.sh"
echo -e "${GREEN}✓${NC} Created check_services.sh"

# Restart script
cat > "$SCRIPT_DIR/restart_services.sh" << 'EOFHELPER'
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
EOFHELPER

chmod +x "$SCRIPT_DIR/restart_services.sh"
echo -e "${GREEN}✓${NC} Created restart_services.sh"

# View logs script
cat > "$SCRIPT_DIR/view_logs.sh" << 'EOFHELPER'
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
EOFHELPER

chmod +x "$SCRIPT_DIR/view_logs.sh"
echo -e "${GREEN}✓${NC} Created view_logs.sh"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Services installed:"
echo "  • rtcm-server.service"
echo "  • teensy-bridge.service"
echo "  • led-controller.service"
echo ""
echo "Helper scripts created in $SCRIPT_DIR:"
echo "  • check_services.sh   - Check service status"
echo "  • restart_services.sh - Restart all services (requires sudo)"
echo "  • view_logs.sh        - View service logs"
echo ""
echo "Useful commands:"
echo "  Check status:  $SCRIPT_DIR/check_services.sh"
echo "  View logs:     $SCRIPT_DIR/view_logs.sh all"
echo "  Restart all:   sudo systemctl restart rtcm-server teensy-bridge led-controller"
echo "  Stop all:      sudo systemctl stop rtcm-server teensy-bridge led-controller"
echo ""
echo "Logs can also be viewed with:"
echo "  sudo journalctl -u rtcm-server -f"
echo "  sudo journalctl -u teensy-bridge -f"
echo "  sudo journalctl -u led-controller -f"
echo ""
echo -e "${YELLOW}Note: Services will start automatically on next boot${NC}"
echo ""
