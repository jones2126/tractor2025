#!/usr/bin/env python3
"""Prepared installer; explicit stationary approval required. Never runs on import."""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
UNIT_DIR = Path("/etc/systemd/system")
UNITS = {
    "calc-publish-position.service": "calc-publish-position.service",
    "tractor-teensy-bridge.service": "tractor-teensy-bridge.service",
    "tractor-telemetry-recorder.service": "tractor-telemetry-recorder.service",
    "tractor-startup-preflight.service": "tractor-startup-preflight-consolidated.service",
}
ALIASES = {"rtcm-server.service": "calc-publish-position.service",
           "tractor-rpi-arbitrator.service": "tractor-teensy-bridge.service"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--stationary-approved", action="store_true",
                        help="Separate approval: stopped Pause, engine off, blades disengaged")
    args = parser.parse_args(argv)
    for target, source in UNITS.items():
        if not (ROOT / "tractor_rpi/setup" / source).is_file():
            parser.error("Missing unit source: " + source)
        print(f"Prepared: {target} <- {source}")
    print("Aliases:", ALIASES)
    if not args.apply:
        print("REVIEW ONLY: no installed unit, service or device changed.")
        return 0
    if not args.stationary_approved or os.geteuid() != 0:
        parser.error("Installation requires root and explicit --stationary-approved")
    git = lambda *a: subprocess.check_output(["git", "-C", str(ROOT), *a], text=True).strip()
    if git("status", "--porcelain") or git("branch", "--show-current") != "main":
        parser.error("Use an exact reviewed, clean main checkout")
    if Path("/run/systemd/system/rtcm-server.service.d/position-candidate.conf").exists():
        parser.error("Temporary candidate override exists; review/restore it first")
    for name in set(UNITS) | set(ALIASES):
        if not any((directory / name).exists() or (directory / name).is_symlink()
                   for directory in (UNIT_DIR, Path("/run/systemd/system"), Path("/lib/systemd/system"))):
            continue
        dropins = subprocess.run(["systemctl", "show", name, "--property=DropInPaths", "--value"],
                                capture_output=True, text=True, check=True).stdout.strip()
        if dropins:
            parser.error(f"Review installed drop-ins before migration: {name}: {dropins}")
    # Capture full effective units and every installed file being replaced first.
    backup = Path("/var/backups") / dt.datetime.now(dt.timezone.utc).strftime("tractor-consolidation-%Y%m%dT%H%M%SZ")
    backup.mkdir(exist_ok=False)
    manifest = dict(commit=git("rev-parse", "HEAD"), units={}, state={})
    names = set(UNITS) | set(ALIASES)
    for name in sorted(names):
        path = UNIT_DIR / name
        manifest["state"][name] = subprocess.run(["systemctl", "is-enabled", name], capture_output=True, text=True).stdout.strip()
        effective = subprocess.run(["systemctl", "cat", name], capture_output=True, text=True)
        (backup / (name + ".effective.txt")).write_text(effective.stdout + effective.stderr)
        if path.is_symlink():
            manifest["units"][name] = dict(kind="symlink", target=os.readlink(path))
        elif path.exists():
            shutil.copy2(path, backup / name)
            manifest["units"][name] = dict(kind="file")
        else:
            manifest["units"][name] = dict(kind="absent")
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    # No firmware, phone-control configuration or legacy mask is changed.
    subprocess.run(["systemctl", "stop", "tractor-startup-preflight.service", "rtcm-server.service",
                    "tractor-rpi-arbitrator.service"], check=True)
    for name in ALIASES:
        path = UNIT_DIR / name
        if path.exists() or path.is_symlink():
            path.unlink()
    for target, source in UNITS.items():
        path = UNIT_DIR / target
        if path.is_symlink():
            path.unlink()
        shutil.copy2(ROOT / "tractor_rpi/setup" / source, path)
    for alias, target in ALIASES.items():
        (UNIT_DIR / alias).symlink_to(UNIT_DIR / target)
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "enable", *UNITS], check=True)
    print("Installed for separately approved stationary activation; services were NOT started.")
    print("Backup and rollback state:", backup)
    print("Start recorder, bridge and producer; inspect calibration/recording, then canonical preflight.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
