#!/usr/bin/env python3
"""Archive one validated site mission into the Git repository.

The planning workspace remains outside Git while a mission is being developed.
After independent validation reports PASS, this tool copies the reviewed
deployment/audit package to:

    tractor_rpi/pure-pursuit/missions/<site-name>/

It also generates a GitHub-renderable README and verifies every copied file by
SHA-256. It never copies the virtual environment, failed builds, angle sweeps,
or other temporary planning artifacts.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path


def read_json(path: Path) -> dict[str, object]:
    try:
        with path.open("r", encoding="utf-8-sig") as handle:
            value = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Could not read JSON file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def number(value: object, digits: int = 2) -> str:
    if value is None:
        return "not reported"
    return f"{float(value):.{digits}f}"


def make_readme(
    site_name: str,
    mission_name: str,
    launcher_name: str,
    launcher_max_speed_mps: float,
    settings: dict[str, object],
    build: dict[str, object],
    validation: dict[str, object],
    mission_hash: str,
) -> str:
    mission_stem = Path(mission_name).stem
    display_name = site_name.replace("_", " ")
    fallback_count = int(build.get("keyhole_boundary_fallback_count", 0))
    angle = number(settings.get("angle_degrees"), 1)
    compass_a = (90.0 - float(settings["angle_degrees"])) % 360.0
    compass_b = (compass_a + 180.0) % 360.0
    lane_spacing = float(settings["lane_spacing_m"])
    radius = float(settings["turn_radius_m"])
    status = str(validation.get("status", "UNKNOWN"))

    return f"""# {display_name} Coverage Mission

**Status: static validation {status} — not yet field approved**

The executable controller file is `{mission_name}`. It contains:

```text
lat lon yaw_rad lookahead_m speed_mps
```

## Mission summary

| Item | Value |
|---|---:|
| Coverage angle | {angle}° math frame |
| Compass stripe headings | {compass_a:.1f}° / {compass_b:.1f}° |
| Lane spacing | {lane_spacing:.4f} m ({lane_spacing / 0.0254:.1f} in) |
| Planning turn radius | {radius:.2f} m |
| Turn policy | {settings.get("stripe_turn_policy", "not recorded")} |
| Waypoints | {int(validation.get("waypoints", 0)):,} |
| Route length | {number(validation.get("route_length_m"), 1)} m |
| Maximum waypoint gap | {number(validation.get("maximum_gap_m"), 2)} m |
| Minimum validated sampled radius | {number(validation.get("minimum_sampled_radius_m"), 2)} m |
| Boundary keyhole fallbacks | {fallback_count} |
| Static validation | {status} |

## Versioned files

| File | Purpose |
|---|---|
| `{mission_name}` | Executable Pure Pursuit mission |
| `{launcher_name}` | RPi launcher for logger plus controller |
| `01_boundary_final.csv` | Reviewed final site polygon |
| `02_plan_settings.json` | Exact planner inputs |
| `02_coverage_segments.csv` | Reviewed coverage rows and execution order |
| `{mission_stem}_audit.csv` | Waypoint geometry and controller parameters |
| `{mission_stem}_build_report.json` | Connector decisions and build provenance |
| `{mission_stem}_validation.json` | Independent static validation results |
| `{mission_stem}_preview.png` | Visual mission checkpoint |
| `{mission_stem}_validation.png` | Visual validation checkpoint |

The settings and reports retain their original planning-machine paths as
provenance. The executable mission itself has no path dependency.

## Integrity

SHA-256 for `{mission_name}`:

```text
{mission_hash}
```

## Visual checkpoints

![Executable mission preview]({mission_stem}_preview.png)

![Independent validation]({mission_stem}_validation.png)

## Field hold point

Static `PASS` confirms file structure, containment, waypoint spacing, yaw,
speed, and sampled curvature. It does not confirm map accuracy, terrain,
mower-deck clearance, controller tracking, or obstacle clearance.

Before normal operation:

1. Pull this exact Git revision onto the tractor RPi.
2. Confirm the mission SHA-256.
3. Perform a controller-load/no-motion check.
4. Review every boundary keyhole fallback.
5. Conduct a supervised low-speed test with RTK Fixed and immediate e-stop
   access.

Initial RPi launch from this directory:

```bash
bash ./{launcher_name}
```

The launcher caps controller speed at {launcher_max_speed_mps:.2f} m/s. A
different explicitly approved cap can be supplied with `--max-speed`, for
example:

```bash
bash ./{launcher_name} --max-speed {launcher_max_speed_mps:.2f}
```

This directory contains exact geographic coordinates. Do not publish it in a
public repository unless that disclosure is intentional.
"""


def make_launcher(
    site_name: str,
    mission_name: str,
    mission_hash: str,
    launcher_max_speed_mps: float,
) -> str:
    mission_stem = Path(mission_name).stem
    validation_name = f"{mission_stem}_validation.json"
    return f"""#!/usr/bin/env bash
# Generated by archive_site_mission_20260724.py for {site_name}.
# Starts field logging in the background and Pure Pursuit in the foreground.
# Run from any directory; every path is resolved relative to this file.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
PURE_PURSUIT_DIR="$(cd "${{SCRIPT_DIR}}/../.." && pwd)"
TRACTOR_RPI_DIR="$(cd "${{PURE_PURSUIT_DIR}}/.." && pwd)"

MISSION="${{SCRIPT_DIR}}/{mission_name}"
VALIDATION="${{SCRIPT_DIR}}/{validation_name}"
CONTROLLER="${{PURE_PURSUIT_DIR}}/pure_pursuit_controller_20260714.py"
LOGGER="${{TRACTOR_RPI_DIR}}/field_test_logger_20260717.py"
EXPECTED_SHA256="{mission_hash}"

for required in "${{MISSION}}" "${{VALIDATION}}" "${{CONTROLLER}}" "${{LOGGER}}"; do
    if [[ ! -f "${{required}}" ]]; then
        echo "ERROR: required file not found: ${{required}}"
        exit 1
    fi
done

if ! grep -Eq '"status"[[:space:]]*:[[:space:]]*"PASS"' "${{VALIDATION}}"; then
    echo "ERROR: validation report does not contain PASS: ${{VALIDATION}}"
    exit 1
fi

ACTUAL_SHA256="$(sha256sum "${{MISSION}}" | awk '{{print $1}}')"
if [[ "${{ACTUAL_SHA256^^}}" != "${{EXPECTED_SHA256}}" ]]; then
    echo "ERROR: mission SHA-256 does not match the archived validated mission."
    echo "Expected: ${{EXPECTED_SHA256}}"
    echo "Actual  : ${{ACTUAL_SHA256^^}}"
    exit 1
fi

LOGGER_PID=""
cleanup() {{
    if [[ -n "${{LOGGER_PID}}" ]] && kill -0 "${{LOGGER_PID}}" 2>/dev/null; then
        echo ""
        echo "Stopping logger (PID ${{LOGGER_PID}})..."
        kill "${{LOGGER_PID}}"
        wait "${{LOGGER_PID}}" 2>/dev/null || true
        echo "Logger stopped."
    fi
}}
trap cleanup EXIT

echo "========================================"
echo "  tractor2025 -- validated site mission"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "  Site       : {site_name}"
echo "  Mission    : {mission_name}"
echo "  SHA-256    : ${{EXPECTED_SHA256}}"
echo "  Speed cap  : {launcher_max_speed_mps:.2f} m/s"
echo "========================================"

echo "Starting field data logger..."
python3 "${{LOGGER}}" &
LOGGER_PID=$!
echo "Logger running (PID ${{LOGGER_PID}})"
echo ""
echo "Waiting for RTK Fixed + valid heading."
echo "Keep the tractor in Manual or Pause until all checks are complete."
echo "Ctrl+C to abort at any time."
echo ""

python3 "${{CONTROLLER}}" \\
    "${{MISSION}}" \\
    --mode live \\
    --ip 127.0.0.1 \\
    --port 6004 \\
    --max-speed {launcher_max_speed_mps:.2f} \\
    "$@"
"""


def archive(args: argparse.Namespace) -> int:
    site_dir = Path(args.site_dir).resolve()
    if not site_dir.is_dir():
        raise ValueError(f"Site working directory does not exist: {site_dir}")

    site_name = args.site_name or site_dir.name
    if not site_name or Path(site_name).name != site_name:
        raise ValueError("--site-name must be one directory name")

    mission_name = args.mission_file or f"{site_name}_mission.txt"
    if Path(mission_name).name != mission_name or not mission_name.endswith(".txt"):
        raise ValueError("--mission-file must be one .txt filename")
    mission_stem = Path(mission_name).stem
    safe_site_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", site_name).strip("_")
    launcher_name = f"run_{safe_site_name}_mission.sh"

    repo_root = (
        Path(args.repo_root).resolve()
        if args.repo_root
        else Path(__file__).resolve().parents[3]
    )
    target = (
        repo_root
        / "tractor_rpi"
        / "pure-pursuit"
        / "missions"
        / site_name
    )
    if target.exists() and not args.replace:
        raise ValueError(
            f"Archive already exists: {target}. Review it, then rerun with "
            "--replace only if this validated mission should supersede it."
        )

    names = [
        "01_boundary_final.csv",
        "02_plan_settings.json",
        "02_coverage_segments.csv",
        mission_name,
        f"{mission_stem}_audit.csv",
        f"{mission_stem}_build_report.json",
        f"{mission_stem}_preview.png",
        f"{mission_stem}_validation.json",
        f"{mission_stem}_validation.png",
    ]
    missing = [name for name in names if not (site_dir / name).is_file()]
    if missing:
        raise ValueError(
            "Required validated artifacts are missing: " + ", ".join(missing)
        )

    settings = read_json(site_dir / "02_plan_settings.json")
    build = read_json(site_dir / f"{mission_stem}_build_report.json")
    validation = read_json(site_dir / f"{mission_stem}_validation.json")
    if validation.get("status") != "PASS":
        raise ValueError(
            "Mission archive requires validation status PASS; found "
            f"{validation.get('status', 'missing')!r}"
        )
    if Path(str(validation.get("mission_file", ""))).name != mission_name:
        raise ValueError(
            "Validation report refers to a different mission file: "
            f"{validation.get('mission_file')!r}"
        )

    target.mkdir(parents=True, exist_ok=True)
    copied = []
    for name in names:
        source = site_dir / name
        destination = target / name
        shutil.copy2(source, destination)
        if sha256(source) != sha256(destination):
            raise ValueError(f"Hash mismatch after copying {name}")
        copied.append(name)

    mission_hash = sha256(target / mission_name)
    launcher_max_speed_mps = float(args.launcher_max_speed_mps)
    if launcher_max_speed_mps <= 0:
        raise ValueError("--launcher-max-speed-mps must be greater than zero")
    launcher = make_launcher(
        site_name,
        mission_name,
        mission_hash,
        launcher_max_speed_mps,
    )
    launcher_path = target / launcher_name
    launcher_path.write_text(launcher, encoding="utf-8", newline="\n")
    launcher_path.chmod(0o755)
    readme = make_readme(
        site_name,
        mission_name,
        launcher_name,
        launcher_max_speed_mps,
        settings,
        build,
        validation,
        mission_hash,
    )
    (target / "README.md").write_text(readme, encoding="utf-8", newline="\n")

    print(f"Validation status : {validation['status']}")
    print(
        f"Archived files    : {len(copied)} plus {launcher_name} and README.md"
    )
    print(f"Mission SHA-256   : {mission_hash}")
    print(f"Git directory     : {target}")
    print(
        "\nNext: inspect README.md and `git status`, then intentionally commit "
        "and push the reviewed mission package."
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Archive a statically validated site mission into Git"
    )
    parser.add_argument("site_dir", help="external per-site planning directory")
    parser.add_argument(
        "--site-name",
        help="Git mission directory name (default: site working directory name)",
    )
    parser.add_argument(
        "--mission-file",
        help="mission filename in site_dir (default: <site-name>_mission.txt)",
    )
    parser.add_argument(
        "--repo-root",
        help="tractor2025 root (default: inferred from this script)",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace files in an existing archive after explicit review",
    )
    parser.add_argument(
        "--launcher-max-speed-mps",
        type=float,
        default=0.30,
        help="controller speed cap embedded in the generated RPi launcher",
    )
    return parser


def main() -> int:
    try:
        return archive(build_parser().parse_args())
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
