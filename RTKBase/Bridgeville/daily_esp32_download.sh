#!/bin/bash
# /home/al/scripts/daily_esp32_download.sh

set -e  # Exit on error

echo "[$(date '+%Y-%m-%d %H:%M:%S')] === ESP32 Daily Download Started ==="

DATA_DIR="/home/al/esp32_data"
SCRIPT_DIR="/home/al/python"

# Create data directory if it doesn't exist
mkdir -p "$DATA_DIR"

# Check if port is in use (optional but helpful)
if fuser /dev/esp32 2>/dev/null; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: /dev/esp32 is in use by another process!"
    lsof 2>/dev/null | grep esp32 || true
fi

# Run the download with --delete flag
# latest filename: esp32_auto_download_20251206.py
cd "$SCRIPT_DIR"
if python3 esp32_auto_download_20251206.py --delete; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] SUCCESS: Download completed"
    
    # Compress old files (older than 30 days)
    OLD_FILES=$(find "$DATA_DIR" -name "*.csv" -mtime +30 2>/dev/null | wc -l)
    if [ "$OLD_FILES" -gt 0 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Compressing $OLD_FILES old CSV files..."
        find "$DATA_DIR" -name "*.csv" -mtime +30 -exec gzip {} \;
    fi
    
    # Delete very old files (older than 365 days)
    VERY_OLD=$(find "$DATA_DIR" -name "*.csv.gz" -mtime +365 2>/dev/null | wc -l)
    if [ "$VERY_OLD" -gt 0 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Deleting $VERY_OLD very old compressed files..."
        find "$DATA_DIR" -name "*.csv.gz" -mtime +365 -delete
    fi
    
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] === ESP32 Daily Download Completed ==="
else
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: Download failed!"
    exit 1
fi
