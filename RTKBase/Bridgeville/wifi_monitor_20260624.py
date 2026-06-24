#!/usr/bin/env python3
"""
WiFi RSSI Monitor for Bridgeville RTK Base Station
- Logs RSSI every 5 minutes to CSV (rolling 7-day window)
- Sends ntfy.sh notification on boot with location, SSID, signal strength
- Sends ntfy.sh notification on SSID change or when signal becomes weak

Usage: runs as systemd service (wifi-monitor.service)
Manual: python3 wifi_monitor_20260624.py
"""

import subprocess
import time
import os
import csv
import math
from datetime import datetime, timedelta

# --- Configuration ---
INTERFACE       = "wlan0"
WATCHED_SSID    = "Pixel_4952"
LOG_FILE        = "/home/al/tractor2025/RTKBase/Bridgeville/wifi_rssi_log.csv"
POLL_INTERVAL   = 300          # seconds between RSSI samples (5 minutes)
RETAIN_DAYS     = 7
NTFY_TOPIC      = "rpi-rtkbase-jones2126"
NTFY_URL        = f"https://ntfy.sh/{NTFY_TOPIC}"
WEAK_THRESHOLD  = -80          # dBm — below this triggers a weak-signal alert
GPS_PORT        = "/dev/ttyACM0"  # F9P USB port — used for lat/lon at boot


# --- Signal strength label ---
def signal_label(rssi):
    if rssi is None:
        return "unknown"
    if rssi >= -65:
        return "strong"
    if rssi >= -80:
        return "medium"
    return "weak"


# --- WiFi info ---
def get_wifi_info():
    """Return (ssid, rssi_dBm) for current wlan0 connection, or (None, None)."""
    try:
        result = subprocess.run(
            ["iwconfig", INTERFACE],
            capture_output=True, text=True, timeout=5
        )
        out = result.stdout
        ssid, rssi = None, None
        for line in out.splitlines():
            if 'ESSID:"' in line:
                ssid = line.split('ESSID:"')[1].split('"')[0]
                if ssid == "":
                    ssid = None
            if "Signal level=" in line:
                raw = line.split("Signal level=")[1].split()[0]
                try:
                    rssi = int(raw.replace("dBm", "").strip())
                except ValueError:
                    pass
        return ssid, rssi
    except Exception:
        return None, None


# --- GPS lat/lon from F9P via NMEA GGA ---
def get_gps_position(timeout=15):
    """
    Read a GGA sentence from the F9P and return (lat, lon) as floats,
    or (None, None) if unavailable. Stops rtcm_server briefly if needed.
    Since rtcm_server holds the port, we read from the daily_position_log.csv
    as a fallback — it's always present and avoids port conflicts.
    """
    # Prefer the daily position log written by rtcm_base_server
    log_path = "/home/al/tractor2025/RTKBase/Bridgeville/daily_position_log.csv"
    try:
        if os.path.exists(log_path):
            with open(log_path, "r") as f:
                lines = [l.strip() for l in f if l.strip() and not l.startswith("Timestamp")]
            if lines:
                last = lines[-1].split(",")
                # Expected columns: Timestamp, lat, lon, alt, fix_quality, num_sats, hdop
                if len(last) >= 3:
                    lat = float(last[1])
                    lon = float(last[2])
                    if lat != 0.0 and lon != 0.0:
                        return lat, lon
    except Exception:
        pass
    return None, None


# --- ntfy notification ---
def notify(title, message, priority="default"):
    """Send a push notification via ntfy.sh."""
    try:
        subprocess.run(
            ["curl", "-s",
             "-H", f"Title: {title}",
             "-H", f"Priority: {priority}",
             "-d", message,
             NTFY_URL],
            timeout=10,
            capture_output=True
        )
    except Exception:
        pass


# --- CSV logging ---
def ensure_csv_header():
    """Create CSV with header if it doesn't exist."""
    if not os.path.exists(LOG_FILE):
        os.makedirs(os.path.dirname(os.path.abspath(LOG_FILE)), exist_ok=True)
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "SSID", "RSSI_dBm", "Signal_Label"])


def append_csv_row(timestamp, ssid, rssi, label):
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, ssid if ssid else "", rssi if rssi is not None else "", label])


def prune_old_rows():
    """Keep only the last RETAIN_DAYS days of data."""
    if not os.path.exists(LOG_FILE):
        return
    cutoff = datetime.now() - timedelta(days=RETAIN_DAYS)
    try:
        with open(LOG_FILE, "r", newline="") as f:
            rows = list(csv.reader(f))
        if not rows:
            return
        header = rows[0]
        kept = [header]
        for row in rows[1:]:
            try:
                ts = datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S")
                if ts >= cutoff:
                    kept.append(row)
            except (ValueError, IndexError):
                kept.append(row)  # keep malformed rows rather than silently drop
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(kept)
    except Exception:
        pass


# --- Boot notification ---
def send_boot_notification():
    ssid, rssi = get_wifi_info()
    label = signal_label(rssi)
    lat, lon = get_gps_position()

    rssi_str = f"{rssi} dBm" if rssi is not None else "unknown"
    ssid_str = ssid if ssid else "none"

    if lat is not None and lon is not None:
        loc_str = f"Lat: {lat:.7f}, Lon: {lon:.7f}"
        maps_url = f"https://maps.google.com/?q={lat},{lon}"
    else:
        loc_str = "Location: unavailable"
        maps_url = ""

    msg = (
        f"WiFi: {ssid_str} | {rssi_str} ({label})\n"
        f"{loc_str}\n"
        f"{maps_url}"
    ).strip()

    notify("rtkbase booted", msg, priority="default")


# --- Main loop ---
def main():
    ensure_csv_header()
    send_boot_notification()

    prev_ssid = None
    prev_weak_alert = False  # track whether we already sent a weak-signal alert

    while True:
        loop_start = time.time()

        ssid, rssi = get_wifi_info()
        label = signal_label(rssi)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        append_csv_row(timestamp, ssid, rssi, label)

        # --- SSID change notification ---
        if prev_ssid is not None and ssid != prev_ssid:
            msg = (
                f"Was: {prev_ssid}\n"
                f"Now: {ssid if ssid else 'disconnected'} | "
                f"{rssi} dBm ({label})"
            )
            notify("rtkbase WiFi changed", msg, priority="high")
            prev_weak_alert = False  # reset weak alert on SSID change

        # --- Weak signal notification (once per weak episode) ---
        is_weak = rssi is not None and rssi < WEAK_THRESHOLD
        if is_weak and not prev_weak_alert:
            msg = (
                f"SSID: {ssid if ssid else 'unknown'}\n"
                f"Signal: {rssi} dBm ({label})\n"
                f"Threshold: {WEAK_THRESHOLD} dBm"
            )
            notify("rtkbase WiFi weak signal", msg, priority="high")
            prev_weak_alert = True
        elif not is_weak:
            prev_weak_alert = False  # reset when signal recovers

        prev_ssid = ssid

        # Prune old rows once per cycle (cheap on a 5-min cycle)
        prune_old_rows()

        # Sleep for remainder of poll interval
        elapsed = time.time() - loop_start
        sleep_time = max(0, POLL_INTERVAL - elapsed)
        time.sleep(sleep_time)


if __name__ == "__main__":
    main()
