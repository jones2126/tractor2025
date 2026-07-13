#!/usr/bin/env python3
"""
WiFi and startup network monitor for the Bridgeville RTK Base Station.

Startup notifications:
1. Wait 60 seconds after service start, then wait until eth0 or wlan0 has a
   usable IPv4 address and an ntfy.sh notification can actually be delivered.
2. Independently wait until ZeroTier has an address, SSH is listening on the
   ZeroTier address, and an active TCP connection can be made to a configured
   always-on ZeroTier peer. Then send a second ntfy.sh notification.

Ongoing monitoring:
- Log wlan0 RSSI every five minutes.
- Retain seven days of CSV data.
- Notify on SSID changes and weak-signal episodes.

Recommended ZeroTier test peer:
Set the environment variable ZEROTIER_TEST_HOST to the ZeroTier IPv4 address
of an always-on peer such as RPi5NAS, and make sure that peer accepts TCP on
ZEROTIER_TEST_PORT (default 22/SSH).
"""

import csv
import ipaddress
import json
import logging
import os
import socket
import subprocess
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Optional, Tuple

# --- Configuration ---
INTERFACE = "wlan0"
LOCAL_INTERFACES = ("eth0", "wlan0")

LOG_FILE = "/home/al/tractor2025/RTKBase/Bridgeville/wifi_rssi_log.csv"
CURRENT_POSITION_FILE = "/home/al/tractor2025/RTKBase/Bridgeville/current_position.json"
STATE_DIR = Path("/home/al/.cache/wifi-monitor")

POLL_INTERVAL = 300             # RSSI samples: 5 minutes
RETAIN_DAYS = 7
WEAK_THRESHOLD = -80            # Alert below -80 dBm

INITIAL_NETWORK_DELAY = 60       # Initial wait after boot/service start
NETWORK_RETRY_INTERVAL = 30
ZEROTIER_RETRY_INTERVAL = 30

NTFY_TOPIC = "rpi-rtkbase-jones2126"
NTFY_URL = f"https://ntfy.sh/{NTFY_TOPIC}"

# Configure this in the systemd unit rather than hard-coding it here:
# Environment=ZEROTIER_TEST_HOST=<always-on-peer-ZeroTier-IP>
# Environment=ZEROTIER_TEST_PORT=22
ZEROTIER_TEST_HOST = os.environ.get("ZEROTIER_TEST_HOST", "").strip() or None
try:
    ZEROTIER_TEST_PORT = int(os.environ.get("ZEROTIER_TEST_PORT", "22"))
except ValueError:
    ZEROTIER_TEST_PORT = 22

BOOT_ID_FILE = Path("/proc/sys/kernel/random/boot_id")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
LOGGER = logging.getLogger("wifi-monitor")


# --- General helpers ---
def run_command(args, timeout=5):
    """Run a command and return CompletedProcess, or None on execution error."""
    try:
        return subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        LOGGER.debug("Command failed to execute: %s: %s", args, exc)
        return None


def current_boot_id() -> str:
    try:
        return BOOT_ID_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        return "unknown-boot"


def notification_already_sent(name: str) -> bool:
    marker = STATE_DIR / f"{name}.boot_id"
    try:
        return marker.read_text(encoding="utf-8").strip() == current_boot_id()
    except OSError:
        return False


def mark_notification_sent(name: str) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    marker = STATE_DIR / f"{name}.boot_id"
    marker.write_text(current_boot_id() + "\n", encoding="utf-8")


# --- Signal strength ---
def signal_label(rssi: Optional[int]) -> str:
    if rssi is None:
        return "unknown"
    if rssi >= -65:
        return "strong"
    if rssi >= -80:
        return "medium"
    return "weak"


# --- Interface/IP information ---
def get_interface_ipv4(interface: str) -> Optional[str]:
    """Return a usable non-link-local IPv4 address for an interface."""
    result = run_command(["ip", "-4", "-o", "addr", "show", "dev", interface])
    if result is None or result.returncode != 0:
        return None

    for line in result.stdout.splitlines():
        fields = line.split()
        try:
            inet_index = fields.index("inet")
            address = str(ipaddress.ip_interface(fields[inet_index + 1]).ip)
            ip_obj = ipaddress.ip_address(address)
            if not ip_obj.is_loopback and not ip_obj.is_link_local:
                return address
        except (ValueError, IndexError):
            continue
    return None


def get_local_ipv4_addresses() -> Dict[str, str]:
    addresses: Dict[str, str] = {}
    for interface in LOCAL_INTERFACES:
        address = get_interface_ipv4(interface)
        if address:
            addresses[interface] = address
    return addresses


def get_zerotier_interface() -> Tuple[Optional[str], Optional[str]]:
    """Return (interface_name, IPv4) for the first ZeroTier interface."""
    result = run_command(["ip", "-4", "-o", "addr", "show"])
    if result is None or result.returncode != 0:
        return None, None

    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        interface = fields[1].rstrip(":").split("@")[0]
        if not interface.startswith("zt"):
            continue
        try:
            inet_index = fields.index("inet")
            address = str(ipaddress.ip_interface(fields[inet_index + 1]).ip)
            return interface, address
        except (ValueError, IndexError):
            continue
    return None, None


# --- WiFi information ---
def get_wifi_info() -> Tuple[Optional[str], Optional[int]]:
    """Return (SSID, RSSI dBm) for wlan0, or (None, None)."""
    # Prefer the modern iw output.
    result = run_command(["iw", "dev", INTERFACE, "link"])
    if result is not None and result.returncode == 0:
        ssid: Optional[str] = None
        rssi: Optional[int] = None
        for raw_line in result.stdout.splitlines():
            line = raw_line.strip()
            if line.startswith("SSID:"):
                ssid = line.split(":", 1)[1].strip() or None
            elif line.startswith("signal:"):
                try:
                    rssi = int(round(float(line.split()[1])))
                except (ValueError, IndexError):
                    pass
        if ssid is not None or rssi is not None:
            return ssid, rssi

    # Fall back to iwconfig for older installations.
    result = run_command(["iwconfig", INTERFACE])
    if result is None or result.returncode != 0:
        return None, None

    ssid = None
    rssi = None
    for line in result.stdout.splitlines():
        if 'ESSID:"' in line:
            try:
                ssid = line.split('ESSID:"', 1)[1].split('"', 1)[0] or None
            except IndexError:
                pass
        if "Signal level=" in line:
            try:
                raw = line.split("Signal level=", 1)[1].split()[0]
                rssi = int(raw.replace("dBm", "").strip())
            except (ValueError, IndexError):
                pass
    return ssid, rssi


# --- GPS position ---
def get_gps_position():
    """Read current position from current_position.json."""
    try:
        with open(CURRENT_POSITION_FILE, "r", encoding="utf-8") as file_handle:
            data = json.load(file_handle)
        lat = data.get("lat")
        lon = data.get("lon")
        if lat is not None and lon is not None:
            return float(lat), float(lon)
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        LOGGER.debug("GPS position unavailable: %s", exc)
    return None, None


# --- ntfy notification ---
def notify(title: str, message: str, priority: str = "default") -> bool:
    """Send an ntfy notification and report whether delivery was accepted."""
    result = run_command(
        [
            "curl",
            "-fsS",
            "--max-time", "10",
            "-H", f"Title: {title}",
            "-H", f"Priority: {priority}",
            "-d", message,
            NTFY_URL,
        ],
        timeout=15,
    )
    if result is None:
        LOGGER.warning("ntfy command could not be executed")
        return False
    if result.returncode != 0:
        error = result.stderr.strip() or f"curl exit code {result.returncode}"
        LOGGER.warning("ntfy delivery failed: %s", error)
        return False
    return True


# --- First startup notification: local network ---
def build_local_network_message(addresses: Dict[str, str]) -> str:
    lines = ["Local network is available:"]
    for interface in LOCAL_INTERFACES:
        lines.append(f"{interface}: {addresses.get(interface, 'no IPv4 address')}")

    ssid, rssi = get_wifi_info()
    label = signal_label(rssi)
    rssi_text = f"{rssi} dBm" if rssi is not None else "unknown"
    lines.append(f"WiFi: {ssid or 'not connected'} | {rssi_text} ({label})")

    lat, lon = get_gps_position()
    if lat is not None and lon is not None:
        lines.append(f"Lat: {lat:.7f}, Lon: {lon:.7f}")
        lines.append(f"https://maps.google.com/?q={lat},{lon}")
    else:
        lines.append("Location: unavailable")

    return "\n".join(lines)


def wait_for_local_network_notification() -> None:
    """Wait for eth0/wlan0 IPv4 and successful ntfy delivery."""
    if notification_already_sent("local-network"):
        LOGGER.info("Local-network startup notification already sent this boot")
        return

    LOGGER.info("Waiting %s seconds before initial network check", INITIAL_NETWORK_DELAY)
    time.sleep(INITIAL_NETWORK_DELAY)

    last_state = None
    while True:
        addresses = get_local_ipv4_addresses()
        state = tuple(sorted(addresses.items()))
        if state != last_state:
            if addresses:
                LOGGER.info("Local IPv4 detected: %s", addresses)
            else:
                LOGGER.info("No usable IPv4 on eth0 or wlan0")
            last_state = state

        if addresses:
            message = build_local_network_message(addresses)
            if notify("rtkbase local network ready", message):
                mark_notification_sent("local-network")
                LOGGER.info("Local-network startup notification sent")
                return

        time.sleep(NETWORK_RETRY_INTERVAL)


# --- Second startup notification: ZeroTier ---
def service_is_active(service_name: str) -> bool:
    result = run_command(["systemctl", "is-active", "--quiet", service_name])
    return result is not None and result.returncode == 0


def route_uses_interface(destination: str, interface: str) -> bool:
    result = run_command(["ip", "-4", "route", "get", destination])
    if result is None or result.returncode != 0:
        return False
    return f"dev {interface}" in result.stdout


def tcp_connect_from(source_ip: str, destination: str, port: int, timeout=4) -> bool:
    """Make a TCP connection using a specified local source address."""
    try:
        destination_ip = socket.gethostbyname(destination)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.bind((source_ip, 0))
            return sock.connect_ex((destination_ip, port)) == 0
    except OSError:
        return False


def zerotier_ready():
    """
    Return (ready, reason/details).

    This checks:
    - zerotier-one service active
    - ZeroTier interface has IPv4
    - route to the configured peer uses that interface
    - SSH is listening on this RPi's ZeroTier address
    - active TCP connection to an always-on ZeroTier peer succeeds
    """
    if not ZEROTIER_TEST_HOST:
        return False, "ZEROTIER_TEST_HOST is not configured"

    if not service_is_active("zerotier-one.service"):
        return False, "zerotier-one.service is not active"

    zt_interface, zt_ip = get_zerotier_interface()
    if not zt_interface or not zt_ip:
        return False, "ZeroTier interface does not yet have IPv4"

    if not route_uses_interface(ZEROTIER_TEST_HOST, zt_interface):
        return False, f"route to {ZEROTIER_TEST_HOST} is not using {zt_interface}"

    # Confirm that this RPi's SSH server is accepting connections on its ZT IP.
    if not tcp_connect_from(zt_ip, zt_ip, 22, timeout=2):
        return False, f"SSH is not listening on {zt_ip}:22"

    # Confirm actual encrypted-overlay traffic to another ZeroTier member.
    if not tcp_connect_from(
        zt_ip,
        ZEROTIER_TEST_HOST,
        ZEROTIER_TEST_PORT,
        timeout=4,
    ):
        return False, (
            f"cannot reach ZeroTier peer {ZEROTIER_TEST_HOST}:"
            f"{ZEROTIER_TEST_PORT} from {zt_ip}"
        )

    details = (
        f"ZeroTier interface: {zt_interface}\n"
        f"rtkbase ZeroTier IP: {zt_ip}\n"
        f"Local SSH: {zt_ip}:22 is listening\n"
        f"Peer test: {ZEROTIER_TEST_HOST}:{ZEROTIER_TEST_PORT} reachable"
    )
    return True, details


def wait_for_zerotier_notification() -> None:
    """Wait in a background thread for verified ZeroTier data-plane access."""
    if notification_already_sent("zerotier"):
        LOGGER.info("ZeroTier startup notification already sent this boot")
        return

    if not ZEROTIER_TEST_HOST:
        LOGGER.warning(
            "ZeroTier readiness notification disabled: set ZEROTIER_TEST_HOST "
            "to an always-on ZeroTier peer IP"
        )
        return

    last_reason = None
    while True:
        ready, details = zerotier_ready()
        if ready:
            if notify("rtkbase ZeroTier ready", details, priority="high"):
                mark_notification_sent("zerotier")
                LOGGER.info("ZeroTier startup notification sent")
                return
        elif details != last_reason:
            LOGGER.info("ZeroTier not ready: %s", details)
            last_reason = details

        time.sleep(ZEROTIER_RETRY_INTERVAL)


# --- CSV logging ---
def ensure_csv_header() -> None:
    """Create CSV with header if it does not exist."""
    if not os.path.exists(LOG_FILE):
        os.makedirs(os.path.dirname(os.path.abspath(LOG_FILE)), exist_ok=True)
        with open(LOG_FILE, "w", newline="", encoding="utf-8") as file_handle:
            writer = csv.writer(file_handle)
            writer.writerow(["Timestamp", "SSID", "RSSI_dBm", "Signal_Label"])


def append_csv_row(timestamp, ssid, rssi, label) -> None:
    with open(LOG_FILE, "a", newline="", encoding="utf-8") as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(
            [
                timestamp,
                ssid if ssid else "",
                rssi if rssi is not None else "",
                label,
            ]
        )


def prune_old_rows() -> None:
    """Keep only the last RETAIN_DAYS days of data."""
    if not os.path.exists(LOG_FILE):
        return

    cutoff = datetime.now() - timedelta(days=RETAIN_DAYS)
    try:
        with open(LOG_FILE, "r", newline="", encoding="utf-8") as file_handle:
            rows = list(csv.reader(file_handle))
        if not rows:
            return

        header = rows[0]
        kept = [header]
        for row in rows[1:]:
            try:
                timestamp = datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S")
                if timestamp >= cutoff:
                    kept.append(row)
            except (ValueError, IndexError):
                kept.append(row)

        with open(LOG_FILE, "w", newline="", encoding="utf-8") as file_handle:
            csv.writer(file_handle).writerows(kept)
    except OSError as exc:
        LOGGER.warning("Could not prune RSSI CSV: %s", exc)


# --- Main loop ---
def main() -> None:
    ensure_csv_header()

    # Requirement 1: do not proceed until a physical interface has IPv4 and
    # ntfy has actually accepted the first notification.
    wait_for_local_network_notification()

    # Requirement 2 runs independently so RSSI logging continues even if an
    # external ZeroTier test peer is unavailable for a long time.
    threading.Thread(
        target=wait_for_zerotier_notification,
        name="zerotier-startup-watch",
        daemon=True,
    ).start()

    prev_ssid = None
    prev_weak_alert = False

    while True:
        loop_start = time.monotonic()

        ssid, rssi = get_wifi_info()
        label = signal_label(rssi)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        append_csv_row(timestamp, ssid, rssi, label)

        # SSID change notification.
        if prev_ssid is not None and ssid != prev_ssid:
            rssi_text = f"{rssi} dBm" if rssi is not None else "unknown"
            message = (
                f"Was: {prev_ssid}\n"
                f"Now: {ssid if ssid else 'disconnected'} | "
                f"{rssi_text} ({label})"
            )
            notify("rtkbase WiFi changed", message, priority="high")
            prev_weak_alert = False

        # Weak-signal notification, once per weak episode.
        is_weak = rssi is not None and rssi < WEAK_THRESHOLD
        if is_weak and not prev_weak_alert:
            message = (
                f"SSID: {ssid if ssid else 'unknown'}\n"
                f"Signal: {rssi} dBm ({label})\n"
                f"Threshold: {WEAK_THRESHOLD} dBm"
            )
            if notify("rtkbase WiFi weak signal", message, priority="high"):
                prev_weak_alert = True
        elif not is_weak:
            prev_weak_alert = False

        prev_ssid = ssid
        prune_old_rows()

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0, POLL_INTERVAL - elapsed))


if __name__ == "__main__":
    main()
