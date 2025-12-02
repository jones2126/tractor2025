#!/bin/bash
# auto_20hz_test.sh — FLASH ONLY

echo "=== FLASHING TEENSY ==="
cd ~/tractor2025/tractor_teensy
pio run -t upload
echo "FLASH DONE. Now run: python3 grok_simple_20hz_test.py"