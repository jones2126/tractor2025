#!/bin/bash
# install_zt_notify.sh
# Installs a boot-time service that sends an ntfy.sh push once ZeroTier has
# been assigned a valid IP address — so you know when the machine is
# reachable for remote (SSH) access.
#
# Usage: bash ~/repos/tractor2025/tractor_rpi/setup/install_zt_notify.sh
#
# What it does:
#   1. Creates /var/log/zt_notify.log with correct permissions
#   2. Writes /usr/local/bin/zt_notify.sh
#   3. Writes /etc/systemd/system/zt-notify.service
#   4. Enables and starts the service
#
# ntfy topic matches the repo-sync installer: rpi-<hostname>-jones2126

set -e

SCRIPT_PATH="/usr/local/bin/zt_notify.sh"
SERVICE_PATH="/etc/systemd/system/zt-notify.service"
LOG_FILE="/var/log/zt_notify.log"
HOSTNAME=$(hostname)
NTFY_TOPIC="rpi-${HOSTNAME}-jones2126"

echo "=== ZeroTier-Ready Notifier Installer ==="
echo "Hostname:   $HOSTNAME"
echo "ntfy topic: $NTFY_TOPIC"
echo ""

# Step 1 — Create log file
echo "[1/4] Creating log file $LOG_FILE ..."
sudo touch "$LOG_FILE"
sudo chown al:al "$LOG_FILE"
echo "      Done."

# Step 2 — Write the notify script
echo "[2/4] Writing $SCRIPT_PATH ..."
sudo tee "$SCRIPT_PATH" > /dev/null << 'SCRIPT'
#!/bin/bash
LOG_FILE="/var/log/zt_notify.log"
HOSTNAME=$(hostname)
NTFY_TOPIC="rpi-${HOSTNAME}-jones2126"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1" >> "$LOG_FILE"
}

notify() {
    local title="$1"
    local msg="$2"
    local priority="$3"
    local tags="$4"
    curl -s \
        -H "Title: $title" \
        -H "Priority: $priority" \
        -H "Tags: $tags" \
        -d "$msg" \
        ntfy.sh/$NTFY_TOPIC
}

# Return the first IPv4 address assigned to a ZeroTier interface (zt*).
# Uses `ip` (no root needed) rather than zerotier-cli (which needs the authtoken).
get_zt_ip() {
    local iface ip
    for iface in $(ip -o link show | awk -F': ' '{print $2}' | grep '^zt'); do
        ip=$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
        if [ -n "$ip" ]; then
            echo "$ip"
            return 0
        fi
    done
    return 1
}

# Poll up to 5 minutes (60 x 5s) for ZeroTier to come up and get an IP.
ZT_IP=""
for i in {1..60}; do
    ZT_IP=$(get_zt_ip) && [ -n "$ZT_IP" ] && break
    sleep 5
done

if [ -n "$ZT_IP" ]; then
    log "[OK] ZeroTier IP $ZT_IP available"
    notify "$HOSTNAME ZeroTier ready" "Reachable at $ZT_IP — you can SSH now." "default" "satellite"
else
    log "[ERROR] No ZeroTier IP after 5 minutes"
    notify "$HOSTNAME ZeroTier FAILED" "No ZeroTier IP after 5 minutes." "high" "warning"
fi
SCRIPT

sudo chmod +x "$SCRIPT_PATH"
echo "      Done."

# Step 3 — Write the systemd service
echo "[3/4] Writing $SERVICE_PATH ..."
sudo tee "$SERVICE_PATH" > /dev/null << SERVICE
[Unit]
Description=Notify via ntfy when ZeroTier has a valid IP
After=network-online.target zerotier-one.service
Wants=network-online.target zerotier-one.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/zt_notify.sh
User=al
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE
echo "      Done."

# Step 4 — Enable and start the service
echo "[4/4] Enabling and starting zt-notify.service ..."
sudo systemctl daemon-reload
sudo systemctl enable zt-notify.service
sudo systemctl start zt-notify.service
echo "      Done."

echo ""
echo "=== Installation complete ==="
echo "Check log:     cat $LOG_FILE"
echo "Check service: sudo systemctl status zt-notify.service"
echo "ntfy topic:    $NTFY_TOPIC"
