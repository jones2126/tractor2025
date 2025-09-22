#!/bin/bash
# Push daily position log to GitHub and prune old gps_log_*.txt files

# Change to the script's directory (where the CSV file is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Move to repo root (two levels up from raspberry-pi/production)
cd ../..

# ---- NEW: Clean up old gps_log_*.txt files ----
LOG_DIR="BridgevilleRTKBase/raspberry-pi/production"
MAX_LOGS=3  # Number of most recent logs to keep

echo "$(date): Cleaning up old gps_log files (keeping last $MAX_LOGS)..."
cd "$LOG_DIR" || { echo "Failed to cd to $LOG_DIR"; exit 1; }

# Find matching files, sort by name (date is in name), delete all but last $MAX_LOGS
FILES_TO_DELETE=$(ls -1 gps_log_*.txt 2>/dev/null | sort | head -n -$MAX_LOGS)
if [ -n "$FILES_TO_DELETE" ]; then
    echo "$FILES_TO_DELETE" | xargs rm -f
    echo "$(date): Deleted old log files:"
    echo "$FILES_TO_DELETE"
else
    echo "$(date): No old log files to delete."
fi

# Return to repo root
cd - >/dev/null

# ---- Git workflow ----
echo "$(date): Pulling latest changes from GitHub..."
git pull origin main

# Path to daily position log
CSV_FILE="BridgevilleRTKBase/raspberry-pi/production/daily_position_log.csv"

if [ ! -f "$CSV_FILE" ]; then
    echo "$(date): CSV file does not exist"
    exit 0
fi

# Check if file is tracked/changed
if git ls-files --error-unmatch "$CSV_FILE" >/dev/null 2>&1; then
    # File is tracked, check for changes
    if git diff --quiet "$CSV_FILE"; then
        echo "$(date): No changes to commit"
        exit 0
    fi
else
    # File is untracked, we should add it
    echo "$(date): CSV file is untracked, adding it"
fi

# Add, commit, and push
git add "$CSV_FILE"
git commit -m "Update daily position log - $(date +%Y-%m-%d)"
git push origin main

echo "$(date): Daily position log pushed to GitHub"
