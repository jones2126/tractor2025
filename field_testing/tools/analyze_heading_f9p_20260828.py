#!/usr/bin/env python3
"""Compare F9P moving-baseline heading with heading derived from RTK positions.

The preferred test has a stationary pause of at least 15 seconds immediately
before and after one perimeter lap. The script detects those pauses, excludes
turning samples, and writes a detailed CSV plus a JSON summary.
"""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


EARTH_RADIUS_M = 6371008.8
SATELLITE_FIELDS = (
    "base_numSV_used",
    "base_numSV_visible",
    "base_cno_mean_dbhz",
    "heading_numSV_used",
    "heading_numSV_visible",
    "heading_cno_mean_dbhz",
)


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def wrap_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


def distance_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(a, b):
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def metric_summary(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return {"count": 0, "min": None, "median": None, "mean": None, "max": None}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "max": max(values),
    }


def error_summary(samples):
    signed = [sample["heading_error_deg"] for sample in samples]
    absolute = [abs(value) for value in signed]
    if not signed:
        return {"samples": 0}
    return {
        "samples": len(signed),
        "signed_mean_deg": statistics.fmean(signed),
        "absolute_median_deg": statistics.median(absolute),
        "absolute_mean_deg": statistics.fmean(absolute),
        "absolute_p90_deg": percentile(absolute, 0.90),
        "absolute_p95_deg": percentile(absolute, 0.95),
        "absolute_max_deg": max(absolute),
    }


def find_stationary_pauses(rows, speed_limit, minimum_duration):
    pauses = []
    start = None
    previous = None
    for row in rows:
        elapsed = number(row.get("elapsed_sec"))
        speed = number(row.get("speed_mps"))
        stationary = elapsed is not None and speed is not None and speed <= speed_limit
        if stationary:
            if start is None or (previous is not None and elapsed - previous > 1.5):
                if start is not None and previous - start >= minimum_duration:
                    pauses.append({"start_sec": start, "end_sec": previous, "duration_sec": previous - start})
                start = elapsed
            previous = elapsed
        elif start is not None:
            if previous - start >= minimum_duration:
                pauses.append({"start_sec": start, "end_sec": previous, "duration_sec": previous - start})
            start = previous = None
    if start is not None and previous - start >= minimum_duration:
        pauses.append({"start_sec": start, "end_sec": previous, "duration_sec": previous - start})
    return pauses


def choose_lap(rows, pauses, start_override, end_override):
    first = number(rows[0].get("elapsed_sec")) or 0.0
    last = number(rows[-1].get("elapsed_sec")) or first
    if start_override is not None or end_override is not None:
        return start_override if start_override is not None else first, end_override if end_override is not None else last, "manual"
    for index, start_pause in enumerate(pauses):
        for end_pause in pauses[index + 1:]:
            if end_pause["start_sec"] - start_pause["end_sec"] >= 20.0:
                return start_pause["end_sec"], end_pause["start_sec"], "stationary pauses"
    return first, last, "entire log (two qualifying pauses not found)"


def unique_position_rows(rows, start_sec, end_sec):
    points = []
    previous = None
    for row in rows:
        elapsed = number(row.get("elapsed_sec"))
        lat, lon = number(row.get("lat")), number(row.get("lon"))
        if elapsed is None or lat is None or lon is None or not start_sec <= elapsed <= end_sec:
            continue
        position = (lat, lon)
        if previous == position:
            continue
        points.append((row, position))
        previous = position
    return points


def derive_heading_samples(points, baseline_m, max_turn_deg, min_speed_mps, require_fixed):
    samples = []
    end_index = 1
    for start_index in range(len(points)):
        if end_index <= start_index:
            end_index = start_index + 1
        while end_index < len(points) and distance_m(points[start_index][1], points[end_index][1]) < baseline_m:
            end_index += 1
        if end_index >= len(points):
            break

        row, start_position = points[start_index]
        speed = number(row.get("speed_mps"))
        heading = number(row.get("heading_deg"))
        if speed is None or speed < min_speed_mps or heading is None or not truthy(row.get("head_valid")):
            continue
        if require_fixed and row.get("fix_quality") != "RTK Fixed":
            continue

        half_index = start_index + 1
        while half_index < end_index and distance_m(start_position, points[half_index][1]) < baseline_m / 2:
            half_index += 1
        if half_index >= end_index:
            continue

        first_bearing = bearing_deg(start_position, points[half_index][1])
        second_bearing = bearing_deg(points[half_index][1], points[end_index][1])
        turn_deg = abs(wrap_degrees(second_bearing - first_bearing))
        if turn_deg > max_turn_deg:
            continue

        derived = bearing_deg(start_position, points[end_index][1])
        sample = {
            "elapsed_sec": number(row.get("elapsed_sec")),
            "carrier": (row.get("carrier") or "unknown").lower(),
            "speed_mps": speed,
            "f9p_heading_deg": heading,
            "position_heading_deg": derived,
            "heading_error_deg": wrap_degrees(heading - derived),
            "turn_over_baseline_deg": turn_deg,
            "baseline_m": distance_m(start_position, points[end_index][1]),
        }
        for field in SATELLITE_FIELDS:
            sample[field] = number(row.get(field))
        samples.append(sample)
    return samples


def carrier_episodes(rows, start_sec, end_sec):
    episodes = []
    current = None
    start = previous = None
    for row in rows:
        elapsed = number(row.get("elapsed_sec"))
        if elapsed is None or not start_sec <= elapsed <= end_sec:
            continue
        carrier = (row.get("carrier") or "unknown").lower()
        if carrier != current:
            if current is not None:
                episodes.append({"carrier": current, "start_sec": start, "end_sec": previous, "duration_sec": max(0.0, previous - start)})
            current, start = carrier, elapsed
        previous = elapsed
    if current is not None:
        episodes.append({"carrier": current, "start_sec": start, "end_sec": previous, "duration_sec": max(0.0, previous - start)})
    return episodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="field_test CSV")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--baseline-m", type=float, default=3.0)
    parser.add_argument("--max-turn-deg", type=float, default=12.0)
    parser.add_argument("--min-speed-mps", type=float, default=0.25)
    parser.add_argument("--pause-speed-mps", type=float, default=0.08)
    parser.add_argument("--pause-duration-sec", type=float, default=15.0)
    parser.add_argument("--start-elapsed", type=float)
    parser.add_argument("--end-elapsed", type=float)
    parser.add_argument("--allow-non-fixed", action="store_true")
    args = parser.parse_args()

    with args.input.open(newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise SystemExit("Input CSV has no data rows")

    required = {"elapsed_sec", "lat", "lon", "heading_deg", "head_valid", "carrier", "speed_mps"}
    missing = sorted(required - set(rows[0]))
    if missing:
        raise SystemExit("Input CSV is missing: " + ", ".join(missing))

    pauses = find_stationary_pauses(rows, args.pause_speed_mps, args.pause_duration_sec)
    start_sec, end_sec, selection = choose_lap(rows, pauses, args.start_elapsed, args.end_elapsed)
    points = unique_position_rows(rows, start_sec, end_sec)
    samples = derive_heading_samples(
        points, args.baseline_m, args.max_turn_deg, args.min_speed_mps, not args.allow_non_fixed
    )

    grouped = {}
    for carrier in sorted({sample["carrier"] for sample in samples}):
        carrier_samples = [sample for sample in samples if sample["carrier"] == carrier]
        grouped[carrier] = error_summary(carrier_samples)
        grouped[carrier]["satellites"] = {
            field: metric_summary([sample[field] for sample in carrier_samples])
            for field in SATELLITE_FIELDS
        }

    selected_rows = [
        row for row in rows
        if (number(row.get("elapsed_sec")) is not None and start_sec <= number(row.get("elapsed_sec")) <= end_sec)
    ]
    summary = {
        "input": str(args.input.resolve()),
        "lap_selection": {"method": selection, "start_sec": start_sec, "end_sec": end_sec},
        "detected_stationary_pauses": pauses,
        "filters": {
            "position_baseline_m": args.baseline_m,
            "maximum_turn_over_baseline_deg": args.max_turn_deg,
            "minimum_speed_mps": args.min_speed_mps,
            "require_rtk_fixed_position": not args.allow_non_fixed,
            "require_heading_valid": True,
        },
        "position_points": len(points),
        "comparison": {"all": error_summary(samples), "by_carrier": grouped},
        "satellites_during_lap": {
            field: metric_summary([number(row.get(field)) for row in selected_rows])
            for field in SATELLITE_FIELDS
        },
        "carrier_episodes": carrier_episodes(rows, start_sec, end_sec),
    }

    output_dir = args.output_dir or args.input.parent / f"{args.input.stem}_heading_analysis_20260828"
    output_dir.mkdir(parents=True, exist_ok=True)
    sample_path = output_dir / "heading_comparison_samples_20260828.csv"
    summary_path = output_dir / "heading_analysis_summary_20260828.json"
    with sample_path.open("w", newline="", encoding="utf-8") as target:
        fieldnames = list(samples[0]) if samples else [
            "elapsed_sec", "carrier", "speed_mps", "f9p_heading_deg",
            "position_heading_deg", "heading_error_deg", "turn_over_baseline_deg", "baseline_m",
            *SATELLITE_FIELDS,
        ]
        writer = csv.DictWriter(target, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"Lap: {start_sec:.2f} to {end_sec:.2f} s ({selection})")
    print(f"Straight comparison samples: {len(samples)}")
    for carrier, stats in grouped.items():
        print(f"  {carrier}: n={stats['samples']}, median |error|={stats.get('absolute_median_deg', float('nan')):.2f} deg, p95={stats.get('absolute_p95_deg', float('nan')):.2f} deg")
    print(f"Summary: {summary_path}")
    print(f"Samples: {sample_path}")


if __name__ == "__main__":
    main()
