#!/bin/bash
# install_repo_sync.sh
# Installs the repo-sync boot check on any tractor RPi.
# Auto-detects repo path from script location — works regardless of where
# the repo is cloned (~/tractor2025 or ~/repos/tractor2025 etc.)
#
# Usage: bash ~/tractor2025/tractor_rpi/setup/install_repo_sync.sh
#
# What it does:
#   1. Creates /var/log/repo_sync.log with correct permissions
#   2. Writes /usr/local/bin/repo_sync_check.sh
#   3. Writes /etc/systemd/system/repo-sync.service
#   4. Enables and starts the service
#   5. Sets git pager to cat

set -e

# Auto-detect repo root from this script's location (works on any machine)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

SCRIPT_PATH="/usr/local/bin/repo_sync_check.sh"
SERVICE_PATH="/etc/systemd/system/repo-sync.service"
LOG_FILE="/var/log/repo_sync.log"
HOSTNAME=$(hostname)
NTFY_TOPIC="rpi-${HOSTNAME}-jones2126"

echo "=== Repo Sync Boot Check Installer ==="
echo "Hostname:   $HOSTNAME"
echo "ntfy topic: $NTFY_TOPIC"
echo "Repo dir:   $REPO_DIR"
echo ""

# Confirm repo dir looks right
if [ ! -f "$REPO_DIR/obsidian_vault/00-project-overview.md" ]; then
    echo "ERROR: $REPO_DIR does not look like the tractor2025 repo."
    echo "Could not find obsidian_vault/00-project-overview.md"
    exit 1
fi

# Step 1 — Create log file
echo "[1/5] Creating log file $LOG_FILE ..."
sudo touch "$LOG_FILE"
sudo chown al:al "$LOG_FILE"
echo "      Done."

# Step 2 — Write the sync script
echo "[2/5] Writing $SCRIPT_PATH ..."
sudo tee "$SCRIPT_PATH" > /dev/null << SCRIPT
#!/bin/bash
REPO_DIR="$REPO_DIR"
LOG_FILE="/var/log/repo_sync.log"
HOSTNAME=\$(hostname)
NTFY_TOPIC="rpi-\${HOSTNAME}-jones2126"

log() {
    echo "\$(date '+%Y-%m-%d %H:%M:%S') \$1" >> "\$LOG_FILE"
}

notify() {
    local title="\$1"
    local msg="\$2"
    local priority="\$3"
    local tags="\$4"
    curl -s \\
        -H "Title: \$title" \\
        -H "Priority: \$priority" \\
        -H "Tags: \$tags" \\
        -d "\$msg" \\
        ntfy.sh/\$NTFY_TOPIC
}

cd "\$REPO_DIR" || {
    log "[ERROR] Could not cd to \$REPO_DIR"
    notify "\$HOSTNAME repo ERROR" "repo dir not found" "high" "warning"
    exit 1
}

# Wait for network up to 30 seconds
for i in {1..10}; do
    ping -c1 -W2 github.com &>/dev/null && break
    sleep 3
done

if ! ping -c1 -W2 github.com &>/dev/null; then
    log "[ERROR] No network after 30s"
    exit 1
fi

# Wait up to 60s for NTP sync
for i in {1..30}; do
    [ "\$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ] && break
    sleep 2
done
if [ "\$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]; then
    log "[WARN] Clock not NTP-synced after 60s; timestamps may be off"
fi

git fetch origin >> "\$LOG_FILE" 2>&1

LOCAL=\$(git rev-parse HEAD)
REMOTE=\$(git rev-parse origin/main)
SHORT_LOCAL=\${LOCAL:0:7}
SHORT_REMOTE=\${REMOTE:0:7}

BEHIND=\$(git rev-list HEAD..origin/main --count)
AHEAD=\$(git rev-list origin/main..HEAD --count)

# Get uncommitted changes — filenames only, one per line
UNCOMMITTED=\$(git status --porcelain)
DIRTY_COUNT=\$(echo "\$UNCOMMITTED" | grep -c . || true)
# Build a short file list for the notification (max 5 files)
DIRTY_FILES=\$(git status --porcelain | awk '{print \$2}' | head -5 | tr '\\n' ' ')

# Build a short list of unpushed commit messages (max 3)
AHEAD_COMMITS=\$(git log origin/main..HEAD --oneline | head -3 | tr '\\n' '|')

if [ "\$LOCAL" = "\$REMOTE" ]; then
    log "[OK] Up to date at \$SHORT_LOCAL"
    if [ -n "\$UNCOMMITTED" ]; then
        log "[DIRTY] \$DIRTY_COUNT uncommitted local changes: \$DIRTY_FILES"
        notify "\$HOSTNAME repo DIRTY" "At \$SHORT_LOCAL but \$DIRTY_COUNT uncommitted changes: \$DIRTY_FILES — run git status on \$HOSTNAME" "default" "pencil"
    else
        notify "\$HOSTNAME repo OK" "Up to date at \$SHORT_LOCAL" "default" "white_check_mark"
    fi

elif [ "\$AHEAD" -gt 0 ] && [ "\$BEHIND" -eq 0 ]; then
    log "[AHEAD] \$AHEAD unpushed commits. Local=\$SHORT_LOCAL Remote=\$SHORT_REMOTE"
    log "[AHEAD] Commits: \$AHEAD_COMMITS"
    if [ -n "\$UNCOMMITTED" ]; then
        log "[DIRTY] Also \$DIRTY_COUNT uncommitted changes: \$DIRTY_FILES"
        notify "\$HOSTNAME repo AHEAD+DIRTY" "\$AHEAD unpushed commits + \$DIRTY_COUNT uncommitted changes on \$HOSTNAME. Push from RPiNAS needed." "default" "arrow_up"
    else
        notify "\$HOSTNAME repo AHEAD" "\$AHEAD unpushed commits on \$HOSTNAME: \$AHEAD_COMMITS Push from RPiNAS needed." "default" "arrow_up"
    fi

elif [ "\$AHEAD" -gt 0 ] && [ "\$BEHIND" -gt 0 ]; then
    log "[DIVERGED] \$AHEAD ahead, \$BEHIND behind. Local=\$SHORT_LOCAL Remote=\$SHORT_REMOTE"
    log "[DIVERGED] Local commits: \$AHEAD_COMMITS"
    notify "\$HOSTNAME repo DIVERGED" "\$AHEAD ahead + \$BEHIND behind. Local=\$SHORT_LOCAL Remote=\$SHORT_REMOTE. Manual fix needed." "high" "warning"

else
    log "[BEHIND] \$BEHIND commits behind. Local=\$SHORT_LOCAL Remote=\$SHORT_REMOTE - pulling"
    git pull origin main >> "\$LOG_FILE" 2>&1
    PULL_STATUS=\$?
    if [ \$PULL_STATUS -eq 0 ]; then
        log "[DONE] Pull successful. Services need manual restart."
        notify "\$HOSTNAME repo UPDATED" "Pulled \$BEHIND commits. Was \$SHORT_LOCAL now \$SHORT_REMOTE. Check services." "default" "arrow_down"
    else
        log "[ERROR] Pull failed"
        notify "\$HOSTNAME repo PULL FAILED" "Pull failed after \$BEHIND commits behind. Manual fix needed." "high" "warning"
    fi
fi
SCRIPT

sudo chmod +x "$SCRIPT_PATH"
echo "      Done."

# Step 3 — Write the systemd service
echo "[3/5] Writing $SERVICE_PATH ..."
sudo tee "$SERVICE_PATH" > /dev/null << SERVICE
[Unit]
Description=Check and sync tractor2025 repo on boot
After=network-online.target time-sync.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/repo_sync_check.sh
User=al
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE
echo "      Done."

# Step 4 — Enable and start the service
echo "[4/5] Enabling and starting repo-sync.service ..."
sudo systemctl daemon-reload
sudo systemctl enable repo-sync.service
sudo systemctl restart repo-sync.service
echo "      Done."

# Step 5 — Set git pager to cat
echo "[5/5] Setting git pager to cat ..."
git config --global core.pager cat
echo "      Done."

echo ""
echo "=== Installation complete ==="
echo "Check log:     cat $LOG_FILE"
echo "Check service: sudo systemctl status repo-sync.service"
echo "ntfy topic:    $NTFY_TOPIC"
echo ""
echo "NOTE: If repo was pulled during this run, manually restart"
echo "      affected services:"
echo "  sudo systemctl restart rtcm-server teensy-bridge led-controller"
