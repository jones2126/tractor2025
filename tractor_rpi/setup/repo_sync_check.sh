#!/bin/bash
REPO_DIR="/home/al/repos/tractor2025"
LOG_FILE="/var/log/repo_sync.log"
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

cd "$REPO_DIR" || {
    log "[ERROR] Could not cd to $REPO_DIR"
    notify "$HOSTNAME repo ERROR" "repo dir not found" "high" "warning"
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

# Wait up to 60s for the clock to be NTP-synced before anything time-stamped.
# The Pi has no battery-backed RTC, so it boots with a stale clock; without
# this the log timestamps (and any committed times) would be wrong.
for i in {1..30}; do
    [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ] && break
    sleep 2
done
if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]; then
    log "[WARN] Clock not NTP-synced after 60s; timestamps may be off"
fi

git fetch origin >> "$LOG_FILE" 2>&1

LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse origin/main)
SHORT_LOCAL=${LOCAL:0:7}
SHORT_REMOTE=${REMOTE:0:7}

# NEW: Count commits in each direction
BEHIND=$(git rev-list HEAD..origin/main --count)
AHEAD=$(git rev-list origin/main..HEAD --count)

if [ "$LOCAL" = "$REMOTE" ]; then
    # UNCHANGED: in sync
    log "[OK] Up to date at $SHORT_LOCAL"
    notify "$HOSTNAME repo OK" "Up to date at $SHORT_LOCAL" "low" "white_check_mark"

elif [ "$AHEAD" -gt 0 ] && [ "$BEHIND" -eq 0 ]; then
    # NEW: local is ahead of remote — do not pull, just warn
    log "[AHEAD] $AHEAD local commits not yet pushed. Local=$SHORT_LOCAL Remote=$SHORT_REMOTE"
    notify "$HOSTNAME repo AHEAD" "$AHEAD unpushed local commits. Local=$SHORT_LOCAL Remote=$SHORT_REMOTE. Push from dev machine needed." "default" "arrow_up"

elif [ "$AHEAD" -gt 0 ] && [ "$BEHIND" -gt 0 ]; then
    # NEW: diverged — local and remote have both moved; do not auto-pull
    log "[DIVERGED] $AHEAD ahead, $BEHIND behind. Local=$SHORT_LOCAL Remote=$SHORT_REMOTE — manual merge needed"
    notify "$HOSTNAME repo DIVERGED" "$AHEAD ahead, $BEHIND behind. Local=$SHORT_LOCAL Remote=$SHORT_REMOTE. Manual fix needed." "high" "warning"

else
    # UNCHANGED path: purely behind — safe to pull
    log "[BEHIND] $BEHIND commits behind. Local=$SHORT_LOCAL Remote=$SHORT_REMOTE - pulling"

    git pull origin main >> "$LOG_FILE" 2>&1
    PULL_STATUS=$?

    if [ $PULL_STATUS -eq 0 ]; then
        log "[DONE] Pull successful. Services need manual restart."
        notify "$HOSTNAME repo UPDATED" "Pulled $BEHIND commits. Was $SHORT_LOCAL now $SHORT_REMOTE. Check services." "default" "arrow_down"
    else
        log "[ERROR] Pull failed"
        notify "$HOSTNAME repo PULL FAILED" "Pull failed after $BEHIND commits behind. Manual fix needed." "high" "warning"
    fi
fi
