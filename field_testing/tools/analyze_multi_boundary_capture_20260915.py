#!/usr/bin/env python3
"""Split a continuous manual field capture into reviewable candidate paths.

Long Pause intervals are treated as separators.  Every sufficiently long
manual-driving interval between those separators is preserved as its own CSV,
summarized in JSON, and plotted on a common local-metre frame.  The output is
for human review only; it does not create an executable mission.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


EARTH_NORTH_M_PER_DEG = 110_540.0
EARTH_EAST_M_PER_DEG = 111_320.0


@dataclass
class Span:
    start: int
    end: int


def numeric(row: dict[str, str], field: str) -> float | None:
    try:
        value = float(row.get(field, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def elapsed(row: dict[str, str]) -> float:
    value = numeric(row, "elapsed_sec")
    if value is None:
        raise ValueError("row has no finite elapsed_sec")
    return value


def is_pause(row: dict[str, str]) -> bool:
    return row.get("steer_mode") == "2" and row.get("steer_state") == "PAUSE"


def find_runs(rows: list[dict[str, str]], predicate) -> list[Span]:
    runs: list[Span] = []
    start: int | None = None
    for index, row in enumerate(rows):
        if predicate(row) and start is None:
            start = index
        elif not predicate(row) and start is not None:
            runs.append(Span(start, index - 1))
            start = None
    if start is not None:
        runs.append(Span(start, len(rows) - 1))
    return runs


def distance_m(a: dict[str, str], b: dict[str, str]) -> float:
    lat1 = float(a["lat"])
    lon1 = float(a["lon"])
    lat2 = float(b["lat"])
    lon2 = float(b["lon"])
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    north = (lat2 - lat1) * EARTH_NORTH_M_PER_DEG
    east = (lon2 - lon1) * EARTH_EAST_M_PER_DEG * math.cos(mean_lat)
    return math.hypot(east, north)


def valid_position(row: dict[str, str]) -> bool:
    return numeric(row, "lat") is not None and numeric(row, "lon") is not None


def unique_positions(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    result: list[dict[str, str]] = []
    last: tuple[float, float] | None = None
    for row in rows:
        if not valid_position(row):
            continue
        position = (float(row["lat"]), float(row["lon"]))
        if position != last:
            result.append(row)
            last = position
    return result


def spaced_positions(rows: list[dict[str, str]], spacing_m: float) -> list[dict[str, str]]:
    if len(rows) <= 2:
        return rows[:]
    result = [rows[0]]
    for row in rows[1:-1]:
        if distance_m(result[-1], row) >= spacing_m:
            result.append(row)
    if rows[-1] is not result[-1]:
        result.append(rows[-1])
    return result


def local_xy(row: dict[str, str], origin_lat: float, origin_lon: float) -> tuple[float, float]:
    lat = float(row["lat"])
    lon = float(row["lon"])
    east = (lon - origin_lon) * EARTH_EAST_M_PER_DEG * math.cos(math.radians(origin_lat))
    north = (lat - origin_lat) * EARTH_NORTH_M_PER_DEG
    return east, north


def polygon_area_m2(rows: list[dict[str, str]], origin_lat: float, origin_lon: float) -> float:
    if len(rows) < 3:
        return 0.0
    points = [local_xy(row, origin_lat, origin_lon) for row in rows]
    total = sum(
        left[0] * right[1] - right[0] * left[1]
        for left, right in zip(points, points[1:] + points[:1])
    )
    return abs(total) / 2.0


def local_time(iso_utc: str) -> str:
    return datetime.fromisoformat(iso_utc).astimezone().isoformat(timespec="milliseconds")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--minimum-pause-seconds", type=float, default=3.0)
    parser.add_argument("--minimum-drive-seconds", type=float, default=8.0)
    parser.add_argument("--minimum-path-metres", type=float, default=5.0)
    parser.add_argument("--point-spacing-metres", type=float, default=0.25)
    parser.add_argument(
        "--title",
        default="Multi-boundary survey",
        help="Site title used on the candidate-segment overview map",
    )
    args = parser.parse_args()

    with args.field_log.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit("ERROR: field log is empty")

    pause_runs = [
        run
        for run in find_runs(rows, is_pause)
        if elapsed(rows[run.end]) - elapsed(rows[run.start]) >= args.minimum_pause_seconds
    ]
    if len(pause_runs) < 2:
        raise SystemExit("ERROR: fewer than two qualifying Pause intervals")

    candidates: list[dict[str, object]] = []
    for left, right in zip(pause_runs, pause_runs[1:]):
        start = left.end + 1
        end = right.start - 1
        if end <= start:
            continue
        section = unique_positions(rows[start : end + 1])
        if len(section) < 2:
            continue
        duration = elapsed(rows[end]) - elapsed(rows[start])
        path_length = sum(distance_m(a, b) for a, b in zip(section, section[1:]))
        if duration < args.minimum_drive_seconds or path_length < args.minimum_path_metres:
            continue
        candidates.append(
            {
                "source_start": start,
                "source_end": end,
                "rows": section,
                "duration_s": duration,
                "path_length_m": path_length,
            }
        )

    if not candidates:
        raise SystemExit("ERROR: no qualifying drive segments found")

    all_positions = [row for candidate in candidates for row in candidate["rows"]]
    origin_lat = sum(float(row["lat"]) for row in all_positions) / len(all_positions)
    origin_lon = sum(float(row["lon"]) for row in all_positions) / len(all_positions)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, object] = {
        "source_file": str(args.field_log.resolve()),
        "source_rows": len(rows),
        "capture_start_utc": rows[0]["time"],
        "capture_end_utc": rows[-1]["time"],
        "qualifying_pause_count": len(pause_runs),
        "candidate_segment_count": len(candidates),
        "origin": {"latitude_deg": origin_lat, "longitude_deg": origin_lon},
        "segments": [],
    }

    colors = plt.get_cmap("tab10")
    figure, axis = plt.subplots(figsize=(12, 10))
    combined_points: list[dict[str, object]] = []

    output_fields = [
        "sequence",
        "source_row",
        "time_utc",
        "time_local",
        "elapsed_sec",
        "lat",
        "lon",
        "fix_quality",
        "heading_deg",
        "speed_mps",
    ]
    source_row_by_id = {id(row): index + 2 for index, row in enumerate(rows)}

    for index, candidate in enumerate(candidates, start=1):
        section = candidate["rows"]
        spaced = spaced_positions(section, args.point_spacing_metres)
        closure = distance_m(section[0], section[-1])
        fixed_count = sum(row.get("fix_quality") == "RTK Fixed" for row in section)
        area = polygon_area_m2(section, origin_lat, origin_lon)
        start_utc = section[0]["time"]
        end_utc = section[-1]["time"]
        path_name = f"segment_{index:02d}_candidate_path.csv"
        path = args.output_dir / path_name
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=output_fields)
            writer.writeheader()
            for sequence, row in enumerate(spaced, start=1):
                writer.writerow(
                    {
                        "sequence": sequence,
                        "source_row": source_row_by_id[id(row)],
                        "time_utc": row["time"],
                        "time_local": local_time(row["time"]),
                        "elapsed_sec": row["elapsed_sec"],
                        "lat": row["lat"],
                        "lon": row["lon"],
                        "fix_quality": row["fix_quality"],
                        "heading_deg": row["heading_deg"],
                        "speed_mps": row["speed_mps"],
                    }
                )

        xy = [local_xy(row, origin_lat, origin_lon) for row in spaced]
        east = [point[0] for point in xy]
        north = [point[1] for point in xy]
        color = colors((index - 1) % 10)
        axis.plot(east, north, linewidth=2.1, color=color, label=f"Segment {index}")
        axis.scatter(east[0], north[0], marker="o", s=35, color=color, zorder=4)
        axis.scatter(east[-1], north[-1], marker="x", s=55, color=color, zorder=4)
        midpoint = len(east) // 2
        axis.annotate(
            str(index),
            (east[midpoint], north[midpoint]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=10,
            fontweight="bold",
            color=color,
        )

        item = {
            "segment": index,
            "candidate_file": path_name,
            "source_start_row": int(candidate["source_start"]) + 2,
            "source_end_row": int(candidate["source_end"]) + 2,
            "start_utc": start_utc,
            "end_utc": end_utc,
            "start_local": local_time(start_utc),
            "end_local": local_time(end_utc),
            "start_elapsed_s": elapsed(section[0]),
            "end_elapsed_s": elapsed(section[-1]),
            "duration_s": candidate["duration_s"],
            "path_length_m": candidate["path_length_m"],
            "closure_gap_m": closure,
            "closed_area_m2": area,
            "raw_unique_positions": len(section),
            "candidate_points": len(spaced),
            "rtk_fixed_fraction": fixed_count / len(section),
        }
        summary["segments"].append(item)
        combined_points.append(
            {
                **item,
                "points": [
                    {
                        "east_m": round(x, 3),
                        "north_m": round(y, 3),
                        "lat": round(float(row["lat"]), 8),
                        "lon": round(float(row["lon"]), 8),
                        "elapsed_s": round(elapsed(row), 2),
                        "time_local": local_time(row["time"]),
                    }
                    for row, (x, y) in zip(spaced, xy)
                ],
            }
        )

    axis.set_title(f"{args.title} — candidate driven segments")
    axis.set_xlabel("East (metres)")
    axis.set_ylabel("North (metres)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    overview = args.output_dir / "candidate_segments_overview.png"
    figure.savefig(overview, dpi=200, bbox_inches="tight")
    plt.close(figure)

    (args.output_dir / "candidate_segments_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    worksheet_path = args.output_dir / "segment_labeling_worksheet.csv"
    worksheet_fields = [
        "segment",
        "label",
        "role",
        "include",
        "start_local",
        "end_local",
        "duration_s",
        "path_length_m",
        "closure_gap_m",
        "closed_area_m2",
        "rtk_fixed_fraction",
        "candidate_file",
        "notes",
    ]
    with worksheet_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=worksheet_fields)
        writer.writeheader()
        for item in summary["segments"]:
            writer.writerow(
                {
                    **{field: item.get(field, "") for field in worksheet_fields},
                    "label": "",
                    "role": "",
                    "include": "",
                    "notes": "",
                }
            )
    (args.output_dir / "candidate_segments_map_data.json").write_text(
        json.dumps({"origin": summary["origin"], "segments": combined_points}, separators=(",", ":")),
        encoding="utf-8",
    )

    print(f"Source rows: {len(rows)}")
    print(f"Qualifying pauses: {len(pause_runs)}")
    print(f"Candidate segments: {len(candidates)}")
    for item in summary["segments"]:
        print(
            f"Segment {item['segment']:02d}: {item['start_local']} to {item['end_local']}; "
            f"{item['duration_s']:.1f}s, {item['path_length_m']:.1f}m, "
            f"closure {item['closure_gap_m']:.2f}m, area {item['closed_area_m2']:.1f}m2, "
            f"RTK Fixed {item['rtk_fixed_fraction']:.1%}"
        )
    print(f"Created: {overview}")
    print(f"Created: {worksheet_path}")


if __name__ == "__main__":
    main()
