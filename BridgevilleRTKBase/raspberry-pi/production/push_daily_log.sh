#!/bin/bash
# Push daily position log to GitHub

# Change to the script's directory (where the CSV file is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Move to repo root (two levels up from raspberry-pi/production)
cd ../..

# Pull latest changes from GitHub first to avoid conflicts
echo "$(date): Pulling latest changes from GitHub..."
git pull origin main

# Check if the CSV file exists and has changes (or is untracked)
CSV_FILE="raspberry-pi/production/daily_position_log.csv"

if [ ! -f "$CSV_FILE" ]; then
    echo "$(date): CSV file does not exist"
    exit 0
fi

# Check if file is untracked or has changes
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

# Add, commit and push
git add "$CSV_FILE"
git commit -m "Update daily position log - $(date +%Y-%m-%d)"
git push origin main

echo "$(date): Daily position log pushed to GitHub"
