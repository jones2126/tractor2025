#!/usr/bin/env python3
"""Recover ZeroTier after an Internet outage at the Bridgeville RTK base."""

from __future__ import annotations

import logging
import os
import subprocess
import time
import urllib.error
import urllib.request


NAS_ZEROTIER_IP = os.environ.get("NAS_ZEROTIER_IP", "192.168.193.217")
NTFY_TOPIC = os.environ.get("NTFY_TOPIC", "rpi-rtkbase-jones2126")
NTFY_URL = f"https://ntfy.sh/{NTFY_TOPIC}"

HEALTHY_CHECK_INTERVAL = int(os.environ.get("HEALTHY_CHECK_INTERVAL", "60"))
INTERNET_RETRY_INTERVAL = int(os.environ.get("INTERNET_RETRY_INTERVAL", "30"))
ZEROTIER_SETTLE_INTERVAL = int(os.environ.get("ZEROTIER_SETTLE_INTERVAL", "15"))
ZEROTIER_SETTLE_ATTEMPTS = int(os.environ.get("ZEROTIER_SETTLE_ATTEMPTS", "20"))
RESTART_RETRY_INTERVAL = int(os.environ.get("RESTART_RETRY_INTERVAL", "300"))
NOTIFY_RETRY_INTERVAL = int(os.environ.get("NOTIFY_RETRY_INTERVAL", "30"))

# More than one target avoids treating a single provider outage as loss of Internet.
INTERNET_TEST_URLS = (
    "https://connectivitycheck.gstatic.com/generate_204",
    "https://www.cloudflare.com/cdn-cgi/trace",
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
LOGGER = logging.getLogger("zerotier-recovery-watchdog")


def run_command(args: list[str], timeout: int = 15) -> subprocess.CompletedProcess[str] | None:
    """Run a command without raising if it fails."""
    try:
        return subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        LOGGER.warning("Could not run %s: %s", args[0], exc)
        return None


def zerotier_service_active() -> bool:
    result = run_command(
        ["systemctl", "is-active", "--quiet", "zerotier-one.service"]
    )
    return result is not None and result.returncode == 0


def nas_reachable() -> bool:
    result = run_command(
        ["ping", "-c", "1", "-W", "3", NAS_ZEROTIER_IP],
        timeout=5,
    )
    return result is not None and result.returncode == 0


def internet_available() -> bool:
    """Return True when at least one public HTTPS endpoint responds."""
    for url in INTERNET_TEST_URLS:
        try:
            request = urllib.request.Request(
                url,
                headers={"User-Agent": "rtkbase-zerotier-watchdog/1.0"},
            )
            with urllib.request.urlopen(request, timeout=8):
                return True
        except urllib.error.HTTPError:
            # An HTTP response still proves that DNS, routing, and TLS work.
            return True
        except (urllib.error.URLError, TimeoutError, OSError):
            continue
    return False


def restart_zerotier() -> bool:
    LOGGER.info("Internet is available; restarting zerotier-one.service")
    result = run_command(
        ["systemctl", "restart", "zerotier-one.service"],
        timeout=30,
    )
    if result is not None and result.returncode == 0:
        return True

    detail = "no command result"
    if result is not None:
        detail = result.stderr.strip() or f"exit status {result.returncode}"
    LOGGER.error("ZeroTier restart failed: %s", detail)
    return False


def send_recovery_notification() -> bool:
    hostname = os.uname().nodename
    message = (
        f"{hostname}: Internet service returned, ZeroTier was restarted, "
        f"and RPi5NAS ({NAS_ZEROTIER_IP}) is reachable again."
    ).encode("utf-8")
    request = urllib.request.Request(
        NTFY_URL,
        data=message,
        method="POST",
        headers={
            "Title": "RTK base ZeroTier restored",
            "Priority": "high",
            "Tags": "white_check_mark,satellite",
            "User-Agent": "rtkbase-zerotier-watchdog/1.0",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            if 200 <= response.status < 300:
                LOGGER.info("Recovery notice sent to ntfy topic %s", NTFY_TOPIC)
                return True
            LOGGER.warning("ntfy returned HTTP %s", response.status)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        LOGGER.warning("Could not send recovery notice: %s", exc)
    return False


def wait_for_internet() -> None:
    first_check = True
    while not internet_available():
        if first_check:
            LOGGER.info("Internet is unavailable; waiting for Starlink/router service")
            first_check = False
        time.sleep(INTERNET_RETRY_INTERVAL)
    LOGGER.info("Internet connectivity confirmed")


def wait_for_zerotier_path() -> bool:
    """Wait a bounded time for the service and NAS data path to recover."""
    for attempt in range(1, ZEROTIER_SETTLE_ATTEMPTS + 1):
        if zerotier_service_active() and nas_reachable():
            LOGGER.info("RPi5NAS is reachable through ZeroTier")
            return True
        if attempt < ZEROTIER_SETTLE_ATTEMPTS:
            time.sleep(ZEROTIER_SETTLE_INTERVAL)
    return False


def recover() -> None:
    """Stay in recovery mode until ZeroTier works and ntfy accepts the notice."""
    while True:
        wait_for_internet()

        if restart_zerotier() and wait_for_zerotier_path():
            while not send_recovery_notification():
                time.sleep(NOTIFY_RETRY_INTERVAL)
            return

        LOGGER.warning(
            "ZeroTier path is still unavailable; another restart will be tried in %s seconds",
            RESTART_RETRY_INTERVAL,
        )
        time.sleep(RESTART_RETRY_INTERVAL)


def main() -> None:
    LOGGER.info(
        "Watching zerotier-one.service and RPi5NAS at %s", NAS_ZEROTIER_IP
    )
    while True:
        service_active = zerotier_service_active()
        peer_reachable = nas_reachable()
        if service_active and peer_reachable:
            time.sleep(HEALTHY_CHECK_INTERVAL)
            continue

        LOGGER.warning(
            "ZeroTier health check failed (service_active=%s, nas_reachable=%s)",
            service_active,
            peer_reachable,
        )
        recover()


if __name__ == "__main__":
    main()
