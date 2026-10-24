#!/usr/bin/env python3
"""Build dashboard-only waypoint files from approved route-concept geometry.

The source still has six join-angle blockers. Output is intentionally named
NOT_FOR_FIELD and must never be used to authorize tractor motion.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "checkpoint_4_detailed_draft_rev002.geojson"
OUT_DIR = HERE / "generated"
MISSION = OUT_DIR / "AREA_OVER_ROAD_review_candidate_NOT_FOR_FIELD_20261024.txt"
AUDIT = OUT_DIR / "AREA_OVER_ROAD_review_candidate_NOT_FOR_FIELD_20261024_audit.csv"
STATUS = "DRAFT - UNTESTED IN THE FIELD"
SPACING_M = 0.20
LAT_M_PER_DEG = 110_540.0
LON_M_PER_DEG = 111_320.0


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    mean_lat = math.radians((a[1] + b[1]) / 2.0)
    return math.hypot((b[0] - a[0]) * LON_M_PER_DEG * math.cos(mean_lat), (b[1] - a[1]) * LAT_M_PER_DEG)


def interpolate(a: tuple[float, float], b: tuple[float, float], fraction: float) -> tuple[float, float]:
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))


def resample(points: list[tuple[float, float]], spacing_m: float) -> list[tuple[float, float]]:
    lengths = [distance_m(a, b) for a, b in zip(points, points[1:])]
    total = sum(lengths)
    if total == 0:
        return [points[0]]
    targets = [i * spacing_m for i in range(int(total // spacing_m) + 1)]
    if total - targets[-1] > 1e-6:
        targets.append(total)
    result: list[tuple[float, float]] = []
    segment = 0
    segment_start = 0.0
    for target in targets:
        while segment < len(lengths) - 1 and target > segment_start + lengths[segment]:
            segment_start += lengths[segment]
            segment += 1
        length = lengths[segment]
        fraction = 0.0 if length == 0 else (target - segment_start) / length
        result.append(interpolate(points[segment], points[segment + 1], max(0.0, min(1.0, fraction))))
    return result


def yaw_rad(a: tuple[float, float], b: tuple[float, float]) -> float:
    mean_lat = math.radians((a[1] + b[1]) / 2.0)
    east = (b[0] - a[0]) * LON_M_PER_DEG * math.cos(mean_lat)
    north = (b[1] - a[1]) * LAT_M_PER_DEG
    return math.atan2(north, east)


def lookahead(segment_type: str) -> float:
    return 1.5 if segment_type in {"headland_turn", "pole_loop", "stripe_with_pole_detour"} else 2.0


def main() -> None:
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    features = sorted(data["features"], key=lambda f: int(f["properties"]["sequence"]))
    if not all(f["properties"].get("status") == STATUS for f in features):
        raise SystemExit("Source status mismatch")
    if not all(f["properties"].get("not_executable") is True for f in features):
        raise SystemExit("Source must remain marked not_executable")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    cumulative = 0.0
    origin = None
    last_point = None
    for feature in features:
        props = feature["properties"]
        points = [tuple(map(float, point[:2])) for point in feature["geometry"]["coordinates"]]
        sampled = resample(points, SPACING_M)
        if last_point and distance_m(last_point, sampled[0]) < 0.01:
            sampled = sampled[1:]
        for index, point in enumerate(sampled):
            if origin is None:
                origin = point
            if last_point is not None:
                cumulative += distance_m(last_point, point)
            if index + 1 < len(sampled):
                heading_to = sampled[index + 1]
            elif last_point is not None:
                heading_to = point
                point_for_yaw = last_point
            else:
                heading_to = point
                point_for_yaw = point
            if index + 1 < len(sampled):
                point_for_yaw = point
            yaw = yaw_rad(point_for_yaw, heading_to) if point_for_yaw != heading_to else 0.0
            mean_lat = math.radians((origin[1] + point[1]) / 2.0)
            east = (point[0] - origin[0]) * LON_M_PER_DEG * math.cos(mean_lat)
            north = (point[1] - origin[1]) * LAT_M_PER_DEG
            rows.append({
                "waypoint": len(rows) + 1,
                "phase": props["phase"],
                "lat": point[1],
                "lon": point[0],
                "yaw_rad": yaw,
                "lookahead_m": lookahead(props["segment_type"]),
                "speed_mps": 1.0,
                "east_m": east,
                "north_m": north,
                "distance_m": cumulative,
                "source_sequence": props["sequence"],
                "source_distance_m": cumulative - float(props["cumulative_start_m"]),
                "source_segment_id": props["segment_id"],
                "source_kind": props["segment_type"],
                "geometry_status": "REVIEW_ONLY_NOT_EXECUTABLE",
            })
            last_point = point

    with MISSION.open("w", encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(
                f"{row['lat']:.9f} {row['lon']:.9f} {row['yaw_rad']:.9f} "
                f"{row['lookahead_m']:.2f} {row['speed_mps']:.2f}\n"
            )
    with AUDIT.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} review-only waypoints; distance {cumulative:.3f} m")
    print(MISSION)
    print(AUDIT)


if __name__ == "__main__":
    main()
