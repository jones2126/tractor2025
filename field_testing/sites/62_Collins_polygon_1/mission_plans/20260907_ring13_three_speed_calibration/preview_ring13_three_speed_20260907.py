#!/usr/bin/env python3
"""Create a PNG preview of the 2026-09-07 Ring 13 three-speed mission."""

from pathlib import Path
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
MISSION = SCRIPT_DIR / "62_Collins_ring13_three_speed_settings_20260907.txt"
OUTPUT = SCRIPT_DIR / "ring13_three_speed_preview_20260907.png"
EXPECTED_ROWS = 506


def main() -> None:
    if not MISSION.is_file():
        raise SystemExit(
            f"ERROR: mission not found: {MISSION}\n"
            "Run build_ring13_three_speed_mission_20260907.py first."
        )

    rows = []
    for line_number, line in enumerate(MISSION.read_text(encoding="utf-8").splitlines(), start=1):
        parts = line.split()
        if len(parts) != 5:
            raise SystemExit(f"ERROR: line {line_number} has {len(parts)} columns; expected 5")
        lat, lon, heading, lookahead, speed = map(float, parts)
        rows.append((lat, lon, heading, lookahead, speed))

    if len(rows) != EXPECTED_ROWS:
        raise SystemExit(f"ERROR: expected {EXPECTED_ROWS} rows, got {len(rows)}")

    lat0, lon0 = rows[0][0], rows[0][1]
    cos_lat = math.cos(math.radians(lat0))

    xy = [
        (
            (lon - lon0) * 111320.0 * cos_lat,
            (lat - lat0) * 110540.0,
        )
        for lat, lon, *_ in rows
    ]

    # 0-based half-open row ranges. The three Ring 13 laps use the same
    # geometry, so draw the fastest lap widest and the slowest lap narrowest;
    # all three remain visible as nested traces instead of one hiding another.
    segments = [
        (0, 66, "Transit: 0.75 m/s", 2.2),
        (359, 506, "Lap 3: 1.50 m/s -> JRK 2200", 6.0),
        (212, 359, "Lap 2: 1.25 m/s -> JRK 2240", 4.0),
        (66, 212, "Lap 1: 1.00 m/s -> JRK 2288", 2.0),
    ]

    fig, ax = plt.subplots(figsize=(10, 9))
    for start, end, label, width in segments:
        points = xy[start:end]
        if start >= 66:
            points = points + [points[0]]
        ax.plot(
            [p[0] for p in points],
            [p[1] for p in points],
            linewidth=width,
            alpha=0.72,
            label=label,
        )

    ax.scatter(xy[0][0], xy[0][1], marker="*", s=120, label="Mission start")
    ax.scatter(xy[66][0], xy[66][1], marker="D", s=60, label="Ring 13 lap start")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East from mission start (m)")
    ax.set_ylabel("North from mission start (m)")
    ax.set_title("2026-09-07 Ring 13 three-speed calibration mission\nSame Ring 13 path repeated for three speed settings")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(OUTPUT, dpi=180)
    plt.close(fig)

    print(f"Created: {OUTPUT}")


if __name__ == "__main__":
    main()
