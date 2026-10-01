#!/usr/bin/env python3
"""Build a review-only interactive replay from Al's mission markup export."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
PACKAGE = SITE / "mission_plans/20260928_outer_perimeter_field_test"
AUDIT = PACKAGE / "generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
INVENTORY = SITE / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
OUTDIR = PACKAGE / "submitted_edits_review_REVIEW_ONLY"
OUT = OUTDIR / "62_Collins_submitted_mission_INTERACTIVE_REVIEW.html"
REPORT = OUTDIR / "62_Collins_submitted_mission_review_report.json"
SOURCE_COPY = OUTDIR / "62_Collins_submitted_markup_SOURCE.json"
ROUTE_EXPORT = OUTDIR / "62_Collins_consolidated_route_REVIEW_ONLY.json"
DEFAULT_EDITS = Path.home() / "Downloads/62_Collins_mission_edits (1).json"

SPACING_M = 0.20
DECK_WIDTH_M = 1.0668
TURN_REVIEW_RADIUS_M = 1.63


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def resample(points, spacing=SPACING_M):
    line = LineString(points)
    count = max(1, math.ceil(line.length / spacing))
    return [[round(v, 3) for v in line.interpolate(i * line.length / count).coords[0]] for i in range(count + 1)]


def chaikin(points, iterations=2):
    result = [tuple(point) for point in points]
    for _ in range(iterations):
        if len(result) < 3:
            break
        refined = [result[0]]
        for a, b in zip(result, result[1:]):
            refined.extend([
                (0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]),
                (0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]),
            ])
        refined.append(result[-1])
        result = refined
    return result


def clean_proposal(points):
    line = LineString(points).simplify(0.10, preserve_topology=False)
    smoothed = chaikin(list(line.coords), 2)
    return resample(smoothed)


def circumradius(a, b, c):
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if cross < 1e-6:
        return math.inf
    return ab * bc * ca / (2.0 * cross)


def turn_warnings(points):
    if len(points) < 11:
        return []
    raw = []
    stride = max(2, round(0.8 / SPACING_M))
    for index in range(stride, len(points) - stride):
        radius = circumradius(points[index - stride], points[index], points[index + stride])
        if radius < TURN_REVIEW_RADIUS_M:
            raw.append((index, radius))
    clustered = []
    for index, radius in raw:
        if clustered and index - clustered[-1][0] < round(1.0 / SPACING_M):
            if radius < clustered[-1][1]:
                clustered[-1] = (index, radius)
        else:
            clustered.append((index, radius))
    return [{"point": points[index], "radius": round(radius, 2)} for index, radius in clustered]


def nearest_mission(mission, point):
    return min(range(len(mission)), key=lambda index: math.dist(mission[index], point))


def local_xy(origin_lat, origin_lon, lat, lon):
    return [
        round((lon - origin_lon) * 111_320.0 * math.cos(math.radians(origin_lat)), 3),
        round((lat - origin_lat) * 110_540.0, 3),
    ]


def build_payload(edits_path):
    edits = json.loads(edits_path.read_text(encoding="utf-8"))
    if edits.get("format") != "tractor2025-mission-markup-v1":
        raise ValueError("unsupported markup format")

    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    mission = [[float(row["east_m"]), float(row["north_m"])] for row in rows]
    origin_lat, origin_lon = float(rows[0]["lat"]), float(rows[0]["lon"])

    erased = {
        index
        for first, last in edits.get("erasedWaypointRanges", [])
        for index in range(max(1, int(first)) - 1, min(len(mission), int(last)))
    }
    accepted = []
    start = None
    for index in range(len(mission) + 1):
        keep = index < len(mission) and index not in erased
        if keep and start is None:
            start = index
        if not keep and start is not None:
            points = mission[start:index]
            if len(points) >= 3 and path_length(points) >= 0.75:
                accepted.append({
                    "id": f"A{len(accepted) + 1}",
                    "label": f"Accepted original W{start + 1}–W{index}",
                    "kind": "accepted",
                    "order": start,
                    "points": [[round(x, 3), round(y, 3)] for x, y in points],
                })
            start = None

    proposed, omitted = [], []
    for number, item in enumerate(edits.get("proposedPaths", []), 1):
        points = [[float(p[0]), float(p[1])] for p in item.get("localEastNorth", [])]
        length = path_length(points) if len(points) > 1 else 0.0
        if len(points) < 5 or length < 1.0:
            omitted.append({"id": f"P{number}", "points": len(points), "length_m": round(length, 2)})
            continue
        cleaned = clean_proposal(points)
        start_anchor = nearest_mission(mission, cleaned[0])
        end_anchor = nearest_mission(mission, cleaned[-1])
        if end_anchor < start_anchor:
            cleaned.reverse()
            start_anchor, end_anchor = end_anchor, start_anchor
        proposed.append({
            "id": f"P{number}",
            "label": f"Submitted path P{number}",
            "kind": "proposed",
            "order": start_anchor,
            "points": cleaned,
            "raw": [[round(x, 3), round(y, 3)] for x, y in points],
            "sourceLengthM": round(length, 2),
            "warnings": turn_warnings(cleaned),
        })

    segments = sorted(accepted + proposed, key=lambda item: (item["order"], 0 if item["kind"] == "accepted" else 1))
    route = []
    cumulative = 0.0
    for segment_index, segment in enumerate(segments):
        for point_index, point in enumerate(segment["points"]):
            if point_index:
                cumulative += math.dist(segment["points"][point_index - 1], point)
            route.append({
                "p": point,
                "d": round(cumulative, 3),
                "sourceD": round(cumulative, 3),
                "segment": segment_index,
                "kind": segment["kind"],
            })

    approved_splices = []

    def connector_entries(start_entry, end_entry, label, source_start, source_end):
        points = resample([start_entry["p"], end_entry["p"]])
        segment_index = len(segments)
        segment_id = f"C{len(approved_splices) + 1}"
        segments.append({
            "id": segment_id,
            "label": label,
            "kind": "approved",
            "order": source_start,
            "points": points,
            "warnings": [],
        })
        approved_splices.append({
            "id": segment_id,
            "label": label,
            "sourceStartM": source_start,
            "sourceEndM": source_end,
            "lengthM": round(path_length(points), 2),
        })
        count = max(1, len(points) - 1)
        return [{
            "p": point,
            "d": 0.0,
            "sourceD": round(source_start + (source_end - source_start) * index / count, 3),
            "segment": segment_index,
            "kind": "approved",
        } for index, point in enumerate(points)]

    def closest_index(items, distance, field="sourceD"):
        return min(range(len(items)), key=lambda index: abs(items[index][field] - distance))

    def shortcut(items, source_start, source_end, label):
        first, last = closest_index(items, source_start), closest_index(items, source_end)
        if first >= last:
            raise ValueError(f"bad shortcut order: {source_start} to {source_end}")
        connector = connector_entries(items[first], items[last], label, source_start, source_end)
        return items[:first + 1] + connector[1:-1] + items[last:]

    # Owner-requested direct replacements, stated against the first review's
    # distance scale.  These eliminate the short red diversion joins.
    direct_shortcuts = [
        (159.8, 161.8),
        (211.1, 212.3),
        (433.3, 434.5),
        (444.7, 446.2),
        (465.3, 466.5),
        (470.4, 471.1),
        (477.0, 483.1),
        (527.3, 532.2),
    ]
    for source_start, source_end in direct_shortcuts:
        route = shortcut(
            route,
            source_start,
            source_end,
            f"Owner-approved direct path: source {source_start:.1f}–{source_end:.1f} m",
        )

    def source_slice(low, high, reverse=False):
        items = [entry for entry in route if low - 0.001 <= entry["sourceD"] <= high + 0.001]
        return list(reversed(items)) if reverse else items

    # Reconstruct the front/over-road transition without discarding the valid
    # submitted geometry.  P9 is deliberately replayed in reverse after the
    # source-751.0 position, followed by P11 and P10 in their submitted order.
    pieces = [
        source_slice(0.0, 547.8),
        source_slice(648.8, 751.0),
        source_slice(548.9, 591.3, reverse=True),
        source_slice(592.2, 604.8),
        source_slice(605.8, 647.9),
        source_slice(751.7, cumulative + 1.0),
    ]
    if any(not piece for piece in pieces):
        raise ValueError("one or more owner-directed route pieces are empty")
    rebuilt = list(pieces[0])
    complex_labels = [
        "Owner-approved direct path: source 547.8–648.8 m",
        "Owner-approved sequence change: source 751.0→591.3 m",
        "Owner-approved direct path: source 548.9→592.2 m",
        "Owner-approved direct path: source 604.8–605.8 m",
        "Owner-approved direct path: source 647.9→751.7 m",
    ]
    for piece, label in zip(pieces[1:], complex_labels):
        start, end = rebuilt[-1], piece[0]
        connector = connector_entries(start, end, label, start["sourceD"], end["sourceD"])
        rebuilt.extend(connector[1:-1])
        rebuilt.extend(piece)
    route = rebuilt

    # Recompute actual replay distance after all shortcuts and reordered pieces.
    cumulative = 0.0
    for index, entry in enumerate(route):
        if index:
            cumulative += math.dist(route[index - 1]["p"], entry["p"])
        entry["d"] = round(cumulative, 3)

    # September 29 second review.  These references use the revised distance
    # scale produced immediately above, not the original source-distance scale.
    for entry in route:
        entry["review2D"] = entry["d"]

    def review_shortcut(items, review_start, review_end):
        first = closest_index(items, review_start, "review2D")
        last = closest_index(items, review_end, "review2D")
        if first >= last:
            raise ValueError(f"bad second-review shortcut order: {review_start} to {review_end}")
        start_entry, end_entry = items[first], items[last]
        label = f"Owner-approved clip: prior review {review_start:.1f}–{review_end:.1f} m"
        connector = connector_entries(
            start_entry, end_entry, label, start_entry["sourceD"], end_entry["sourceD"]
        )
        for index, entry in enumerate(connector):
            entry["review2D"] = round(
                review_start + (review_end - review_start) * index / max(1, len(connector) - 1), 3
            )
        approved_splices[-1]["priorReviewStartM"] = review_start
        approved_splices[-1]["priorReviewEndM"] = review_end
        return items[:first + 1] + connector[1:-1] + items[last:]

    for review_start, review_end in [
        (19.9, 21.2),
        (24.6, 26.1),
        (36.3, 36.9),
        (47.1, 48.2),
        (59.6, 60.3),
        (68.2, 69.1),
        (166.9, 168.7),
    ]:
        route = review_shortcut(route, review_start, review_end)

    def review_slice(low, high, reverse=False):
        items = [entry for entry in route if low - 0.001 <= entry["review2D"] <= high + 0.001]
        result = [dict(entry) for entry in items]
        return list(reversed(result)) if reverse else result

    def segment_slice(first_id, last_id):
        first = next(index for index, entry in enumerate(route) if segments[entry["segment"]]["id"] == first_id)
        last = max(index for index, entry in enumerate(route) if segments[entry["segment"]]["id"] == last_id)
        return [dict(entry) for entry in route[first:last + 1]]

    p17_index = next(index for index, segment in enumerate(segments) if segment["id"] == "P17")
    p17_forward = [{
        "p": point,
        "d": 0.0,
        "sourceD": round(527.3 + (532.2 - 527.3) * index / max(1, len(segments[p17_index]["points"]) - 1), 3),
        "review2D": 490.0,
        "segment": p17_index,
        "kind": "proposed",
    } for index, point in enumerate(segments[p17_index]["points"])]

    second_pieces = [
        review_slice(0.0, 567.1),
        review_slice(464.7, 490.0),
        [dict(entry) for entry in p17_forward],
        review_slice(598.7, 603.8, reverse=True),
        segment_slice("P15", "P13"),
        review_slice(603.8, 606.3, reverse=True),
        list(reversed([dict(entry) for entry in p17_forward])),
    ]
    suffix = segment_slice("A14", "A15")
    second_pieces.append(suffix)
    if any(not piece for piece in second_pieces):
        raise ValueError("one or more second-review route pieces are empty")

    route = list(second_pieces[0])
    second_labels = [
        "Repeat prior-review path 464.7–490.0 m",
        "Follow submitted green access path P17",
        "Reverse prior-review path 598.7–603.8 m",
        "Continue around the over-road polygon",
        "Reverse prior-review ending path 603.8–606.3 m",
        "Repeat submitted green access path P17 in reverse",
        "Unresolved connection to the retained return path",
    ]
    for piece, label in zip(second_pieces[1:], second_labels):
        start, end = route[-1], piece[0]
        gap = math.dist(start["p"], end["p"])
        if label.startswith("Unresolved"):
            route.extend(piece)
            continue
        connector = connector_entries(start, end, label, start["sourceD"], end["sourceD"])
        approved_splices[-1]["secondReviewSequence"] = True
        route.extend(connector[1:-1])
        route.extend(piece)

    cumulative = 0.0
    for index, entry in enumerate(route):
        if index:
            cumulative += math.dist(route[index - 1]["p"], entry["p"])
        entry["d"] = round(cumulative, 3)

    # September 29 third review: remove four more local diversions, then use
    # the cleaned access path in reverse and finish on P21 + A15 to parking.
    for entry in route:
        entry["review3D"] = entry["d"]

    def third_review_shortcut(items, review_start, review_end):
        first = closest_index(items, review_start, "review3D")
        last = closest_index(items, review_end, "review3D")
        if first >= last:
            raise ValueError(f"bad third-review shortcut order: {review_start} to {review_end}")
        start_entry, end_entry = items[first], items[last]
        label = f"Owner-approved clip: third review {review_start:.1f}–{review_end:.1f} m"
        connector = connector_entries(
            start_entry, end_entry, label, start_entry["sourceD"], end_entry["sourceD"]
        )
        for index, entry in enumerate(connector):
            entry["review3D"] = round(
                review_start + (review_end - review_start) * index / max(1, len(connector) - 1), 3
            )
        approved_splices[-1]["thirdReviewStartM"] = review_start
        approved_splices[-1]["thirdReviewEndM"] = review_end
        return items[:first + 1] + connector[1:-1] + items[last:]

    for review_start, review_end in [
        (587.0, 587.6),
        (603.3, 605.3),
        (618.5, 618.8),
        (622.7, 623.9),
    ]:
        route = third_review_shortcut(route, review_start, review_end)

    def third_review_slice(low, high, reverse=False):
        items = [entry for entry in route if low - 0.001 <= entry["review3D"] <= high + 0.001]
        result = [dict(entry) for entry in items]
        return list(reversed(result)) if reverse else result

    p21_index = next(index for index, segment in enumerate(segments) if segment["id"] == "P21")
    p21_forward = [{
        "p": point,
        "d": 0.0,
        "sourceD": round(764.9 + (769.5 - 764.9) * index / max(1, len(segments[p21_index]["points"]) - 1), 3),
        "review3D": 764.2,
        "segment": p21_index,
        "kind": "proposed",
    } for index, point in enumerate(segments[p21_index]["points"])]

    third_pieces = [
        third_review_slice(0.0, 764.2),
        third_review_slice(569.7, 588.8, reverse=True),
        p21_forward,
        segment_slice("A15", "A15"),
    ]
    if any(not piece for piece in third_pieces):
        raise ValueError("one or more third-review route pieces are empty")

    route = list(third_pieces[0])
    third_labels = [
        "Reverse cleaned third-review path 569.7–588.8 m",
        "Follow previously defined parking connector P21",
        "Join P21 to the retained final parking path A15",
    ]
    for piece, label in zip(third_pieces[1:], third_labels):
        start, end = route[-1], piece[0]
        connector = connector_entries(start, end, label, start["sourceD"], end["sourceD"])
        approved_splices[-1]["thirdReviewSequence"] = True
        route.extend(connector[1:-1])
        route.extend(piece)

    cumulative = 0.0
    for index, entry in enumerate(route):
        if index:
            cumulative += math.dist(route[index - 1]["p"], entry["p"])
        entry["d"] = round(cumulative, 3)

    # Final review clips at the parking return, stated on the third review's
    # distance scale.
    for entry in route:
        entry["review4D"] = entry["d"]

    def fourth_review_shortcut(items, review_start, review_end):
        first = closest_index(items, review_start, "review4D")
        last = closest_index(items, review_end, "review4D")
        if first >= last:
            raise ValueError(f"bad fourth-review shortcut order: {review_start} to {review_end}")
        start_entry, end_entry = items[first], items[last]
        label = f"Owner-approved clip: final review {review_start:.1f}–{review_end:.1f} m"
        connector = connector_entries(
            start_entry, end_entry, label, start_entry["sourceD"], end_entry["sourceD"]
        )
        for index, entry in enumerate(connector):
            entry["review4D"] = round(
                review_start + (review_end - review_start) * index / max(1, len(connector) - 1), 3
            )
        approved_splices[-1]["finalReviewStartM"] = review_start
        approved_splices[-1]["finalReviewEndM"] = review_end
        return items[:first + 1] + connector[1:-1] + items[last:]

    for review_start, review_end in [(782.0, 782.5), (787.3, 789.0)]:
        route = fourth_review_shortcut(route, review_start, review_end)

    cumulative = 0.0
    for index, entry in enumerate(route):
        if index:
            cumulative += math.dist(route[index - 1]["p"], entry["p"])
        entry["d"] = round(cumulative, 3)

    joins = []
    for previous, current in zip(route, route[1:]):
        gap = math.dist(previous["p"], current["p"])
        if gap > 0.35:
            joins.append({
                "from": previous["p"],
                "to": current["p"],
                "gapM": round(gap, 2),
                "fromId": segments[previous["segment"]]["id"],
                "toId": segments[current["segment"]]["id"],
            })

    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    obstacles = []
    for feature in inventory["features"]:
        props = feature.get("properties", {})
        if props.get("geometry_role") not in {"candidate_mowing_exclusion", "mowing_exclusion"}:
            continue
        geometry = feature["geometry"]
        polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        for polygon in polygons:
            rings = [
                [local_xy(origin_lat, origin_lon, lat, lon) for lon, lat in ring]
                for ring in polygon
            ]
            outer = rings[0][:-1] or rings[0]
            obstacles.append({
                "id": props.get("asset_id", feature.get("id", "obstacle")),
                "rings": rings,
                "center": [
                    round(sum(point[0] for point in outer) / len(outer), 3),
                    round(sum(point[1] for point in outer) / len(outer), 3),
                ],
            })

    all_points = mission + [point for item in proposed for point in item["points"]]
    bounds = [
        min(point[0] for point in all_points), min(point[1] for point in all_points),
        max(point[0] for point in all_points), max(point[1] for point in all_points),
    ]
    return {
        "title": "62 Collins submitted perimeter mission — INTERACTIVE REVIEW ONLY",
        "source": edits.get("source"),
        "sourceFile": edits_path.name,
        "sourceSha256": hashlib.sha256(edits_path.read_bytes()).hexdigest(),
        "originLat": origin_lat,
        "originLon": origin_lon,
        "deckWidthM": DECK_WIDTH_M,
        "turnReviewRadiusM": TURN_REVIEW_RADIUS_M,
        "segments": segments,
        "route": route,
        "joins": joins,
        "approvedSplices": approved_splices,
        "obstacles": obstacles,
        "bounds": bounds,
        "omitted": omitted,
        "summary": {
            "acceptedSegments": len(accepted),
            "proposedSegments": len(proposed),
            "omittedMarks": len(omitted),
            "routeLengthM": round(cumulative, 1),
            "reviewJoins": sum(join["gapM"] > 1.0 for join in joins),
            "approvedSplices": len(approved_splices),
            "tightTurnMarkers": sum(len(item.get("warnings", [])) for item in proposed),
        },
    }


TEMPLATE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>62 Collins submitted mission review</title>
<style>
:root{color-scheme:light dark;font-family:system-ui,sans-serif;--bg:#f6f8fb;--panel:#fff;--text:#17212b;--muted:#5f6f7f;--border:#b6c1cb;--accepted:#1565c0;--raw:#8e61d1;--candidate:#0a9b57;--obstacle:#ef8b16;--warning:#d92d20;--actual:#00897b} @media(prefers-color-scheme:dark){:root{--bg:#0d141b;--panel:#17212b;--text:#eef3f7;--muted:#aab8c5;--border:#536170;--accepted:#62adff;--raw:#c99aff;--candidate:#48da88;--obstacle:#ffc078;--warning:#ff766c;--actual:#45d3bd}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text)}header{padding:10px 14px;background:var(--panel);border-bottom:1px solid var(--border)}h1{font-size:1.1rem;margin:0 0 3px}.sub{font-size:.84rem;color:var(--muted)}.bar{display:flex;gap:7px;flex-wrap:wrap;align-items:center;padding:8px 12px;background:var(--panel);border-bottom:1px solid var(--border)}button,select{font:inherit;color:var(--text);background:var(--panel);border:1px solid var(--border);border-radius:5px;padding:7px 10px}button.primary{background:var(--text);color:var(--panel)}label{display:flex;gap:5px;align-items:center;font-size:.83rem}.timeline{flex:1 1 260px;min-width:180px}.timeline input{width:100%}.stats{display:flex;gap:14px;flex-wrap:wrap;padding:7px 12px;font-size:.82rem;color:var(--muted);background:var(--panel);border-bottom:1px solid var(--border)}.stats b{color:var(--text);font-weight:500}.map-wrap{height:calc(100vh - 190px);min-height:480px;position:relative}canvas{width:100%;height:100%;display:block;touch-action:none}.status{position:absolute;left:10px;bottom:10px;max-width:min(720px,calc(100% - 20px));background:color-mix(in srgb,var(--panel) 94%,transparent);border:1px solid var(--border);padding:7px 9px;border-radius:5px;font-size:.8rem}.legend{position:absolute;right:10px;top:10px;background:color-mix(in srgb,var(--panel) 94%,transparent);border:1px solid var(--border);padding:7px 9px;border-radius:5px;font-size:.78rem;display:grid;gap:4px}.sw{display:inline-block;width:18px;height:3px;margin-right:5px;vertical-align:middle}.note{padding:6px 12px;background:var(--panel);border-top:1px solid var(--border);font-size:.8rem;color:var(--muted)}@media(max-width:720px){.map-wrap{height:64vh}.legend{font-size:.72rem}.status{font-size:.75rem}}
</style></head><body>
<header><h1>62 Collins submitted perimeter mission</h1><div class="sub">Interactive reconstruction from the exported edits · REVIEW ONLY — not a driveable mission</div></header>
<div class="bar">
 <button id="play" class="primary">▶ Play</button><button id="back">−10 m</button><button id="forward">+10 m</button><button id="reset">Reset</button>
 <label>Playback <select id="rate"><option value="1">1×</option><option value="5">5×</option><option value="10" selected>10×</option><option value="20">20×</option></select></label>
 <label>Focus <select id="focus"><option value="all">Full mission</option><option value="proposed">Submitted paths</option></select></label><button id="fit">Fit view</button>
 <label class="timeline"><span id="distanceLabel">0 m</span><input id="timeline" type="range" min="0" step="0.1" value="0"></label>
</div>
<div class="bar">
 <label><input id="accepted" type="checkbox" checked> Accepted blue</label><label><input id="raw" type="checkbox" checked> Submitted purple</label><label><input id="candidate" type="checkbox" checked> Smoothed green</label><label><input id="obstacles" type="checkbox" checked> Obstacles</label><label><input id="joins" type="checkbox" checked> Unresolved joins</label><label><input id="turns" type="checkbox" checked> Turn review</label><label><input id="deck" type="checkbox" checked> 42-inch deck reference</label>
</div>
<div class="stats"><span>Accepted segments <b id="acceptedCount"></b></span><span>Submitted segments <b id="proposedCount"></b></span><span>Approved splices <b id="spliceCount"></b></span><span>Route geometry <b id="routeLength"></b></span><span>Joins over 1 m <b id="joinCount"></b></span><span>Turn-review markers <b id="turnCount"></b></span></div>
<div class="map-wrap"><canvas id="map" role="img" aria-label="Interactive map and replay of the submitted perimeter mission"></canvas><div class="legend"><span><i class="sw" style="background:var(--accepted)"></i>Accepted original</span><span><i class="sw" style="background:var(--raw)"></i>Your submitted stroke</span><span><i class="sw" style="background:var(--candidate)"></i>Smoothed review path</span><span><i class="sw" style="background:var(--warning)"></i>Unresolved join / tight turn</span></div><div id="status" class="status" aria-live="polite"></div></div>
<div class="note">Playback follows the inferred original mission order. Red dashed joins and red × markers require resolution before a field mission is generated. The deck circle is a width reference centered on the GPS path; the physical GPS-to-deck offset is not modeled here.</div>
<script>
const D=__DATA__,C=document.getElementById('map'),X=C.getContext('2d'),$=id=>document.getElementById(id);let scale=1,ox=0,oy=0,drag=null,playing=false,lastFrame=null,distance=0;
const colors=()=>{const s=getComputedStyle(document.documentElement);return Object.fromEntries(['bg','text','muted','accepted','raw','candidate','obstacle','warning','actual'].map(k=>[k,s.getPropertyValue('--'+k).trim()]))};
function resize(){const r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=Math.round(r.width*d);C.height=Math.round(r.height*d);X.setTransform(d,0,0,d,0,0);draw()}
function fit(points=null){const r=C.getBoundingClientRect(),b=points&&points.length?[Math.min(...points.map(p=>p[0])),Math.min(...points.map(p=>p[1])),Math.max(...points.map(p=>p[0])),Math.max(...points.map(p=>p[1]))]:D.bounds,w=Math.max(1,b[2]-b[0]),h=Math.max(1,b[3]-b[1]),pad=35;scale=Math.min((r.width-pad*2)/w,(r.height-pad*2)/h);ox=r.width/2-(b[0]+b[2])/2*scale;oy=r.height/2+(b[1]+b[3])/2*scale;draw()}
const pt=p=>[ox+p[0]*scale,oy-p[1]*scale];
function line(points,color,width=2,dash=[]){if(points.length<2)return;X.save();X.beginPath();let q=pt(points[0]);X.moveTo(q[0],q[1]);for(let i=1;i<points.length;i++){q=pt(points[i]);X.lineTo(q[0],q[1])}X.strokeStyle=color;X.lineWidth=width;X.setLineDash(dash);X.lineCap='round';X.lineJoin='round';X.stroke();X.restore()}
function polygon(rings,c){for(const ring of rings){X.beginPath();let q=pt(ring[0]);X.moveTo(...q);for(let i=1;i<ring.length;i++)X.lineTo(...pt(ring[i]));X.closePath();X.save();X.globalAlpha=.15;X.fillStyle=c.obstacle;X.fill();X.restore();X.strokeStyle=c.obstacle;X.lineWidth=1.2;X.stroke()}}
function routeIndex(d){let lo=0,hi=D.route.length-1;while(lo<hi){let m=(lo+hi)>>1;if(D.route[m].d<d)lo=m+1;else hi=m}return lo}
function marker(c){if(!D.route.length)return;const i=routeIndex(distance),r=D.route[i],prev=D.route[Math.max(0,i-1)],next=D.route[Math.min(D.route.length-1,i+1)],q=pt(r.p),heading=Math.atan2(next.p[1]-prev.p[1],next.p[0]-prev.p[0]);if($('deck').checked){X.save();X.beginPath();X.arc(q[0],q[1],D.deckWidthM/2*scale,0,Math.PI*2);X.fillStyle=c.candidate;X.globalAlpha=.10;X.fill();X.globalAlpha=.8;X.strokeStyle=c.candidate;X.lineWidth=1.5;X.stroke();X.restore()}X.beginPath();X.arc(q[0],q[1],6,0,Math.PI*2);X.fillStyle=r.kind==='proposed'||r.kind==='approved'?c.candidate:c.accepted;X.fill();X.strokeStyle=c.bg;X.lineWidth=2;X.stroke();X.beginPath();X.moveTo(q[0],q[1]);X.lineTo(q[0]+Math.cos(heading)*22,q[1]-Math.sin(heading)*22);X.strokeStyle=c.text;X.lineWidth=2.5;X.stroke();$('status').textContent=`${D.segments[r.segment].label} · revised ${distance.toFixed(1)} / ${D.summary.routeLengthM.toFixed(1)} m · source reference ${r.sourceD.toFixed(1)} m · approximately ${Math.round(distance)} s at 1.0 m/s · heading ${(90-heading*180/Math.PI+360)%360|0}°`;}
function draw(){const r=C.getBoundingClientRect(),c=colors();X.fillStyle=c.bg;X.fillRect(0,0,r.width,r.height);if($('obstacles').checked){for(const o of D.obstacles){polygon(o.rings,c);const q=pt(o.center);X.fillStyle=c.text;X.font='11px system-ui';X.textAlign='center';X.fillText(o.id,q[0],q[1]-5)}}for(const s of D.segments){if(s.kind==='accepted'&&$('accepted').checked)line(s.points,c.accepted,1.8);if(s.kind==='proposed'){if($('raw').checked)line(s.raw,c.raw,1.3);if($('candidate').checked)line(s.points,c.candidate,2.8)}if(s.kind==='approved'&&$('candidate').checked)line(s.points,c.candidate,2.8)}if($('joins').checked)for(const j of D.joins){if(j.gapM<=.35)continue;line([j.from,j.to],c.warning,j.gapM>1?1.8:1,[6,5])}if($('turns').checked)for(const s of D.segments)for(const w of s.warnings||[]){const q=pt(w.point),z=5;X.strokeStyle=c.warning;X.lineWidth=2;X.beginPath();X.moveTo(q[0]-z,q[1]-z);X.lineTo(q[0]+z,q[1]+z);X.moveTo(q[0]+z,q[1]-z);X.lineTo(q[0]-z,q[1]+z);X.stroke()}marker(c)}
function setDistance(value){distance=Math.max(0,Math.min(D.summary.routeLengthM,Number(value)));$('timeline').value=distance;$('distanceLabel').textContent=distance.toFixed(1)+' m';draw()}
function stop(){playing=false;lastFrame=null;$('play').textContent='▶ Play'}
$('play').onclick=()=>{if(playing){stop();return}if(distance>=D.summary.routeLengthM)setDistance(0);playing=true;$('play').textContent='❚❚ Pause';requestAnimationFrame(animate)};function animate(t){if(!playing)return;if(lastFrame!=null)setDistance(distance+(t-lastFrame)/1000*Number($('rate').value));lastFrame=t;if(distance>=D.summary.routeLengthM){stop();return}requestAnimationFrame(animate)}
$('back').onclick=()=>{stop();setDistance(distance-10)};$('forward').onclick=()=>{stop();setDistance(distance+10)};$('reset').onclick=()=>{stop();setDistance(0)};$('timeline').max=D.summary.routeLengthM;$('timeline').oninput=e=>{stop();setDistance(e.target.value)};$('fit').onclick=()=>fit();$('focus').onchange=e=>{if(e.target.value==='proposed')fit(D.segments.filter(s=>s.kind==='proposed').flatMap(s=>s.points));else fit()};document.querySelectorAll('input[type=checkbox]').forEach(e=>e.onchange=draw);
C.addEventListener('wheel',e=>{e.preventDefault();const r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15,n=Math.max(.3,Math.min(60,scale*k));ox=mx-(mx-ox)*n/scale;oy=my-(my-oy)*n/scale;scale=n;draw()},{passive:false});C.addEventListener('pointerdown',e=>{C.setPointerCapture(e.pointerId);drag=[e.clientX,e.clientY]});C.addEventListener('pointermove',e=>{if(!drag)return;ox+=e.clientX-drag[0];oy+=e.clientY-drag[1];drag=[e.clientX,e.clientY];draw()});C.addEventListener('pointerup',()=>drag=null);
$('acceptedCount').textContent=D.summary.acceptedSegments;$('proposedCount').textContent=D.summary.proposedSegments;$('spliceCount').textContent=D.summary.approvedSplices;$('routeLength').textContent=D.summary.routeLengthM.toFixed(1)+' m';$('joinCount').textContent=D.summary.reviewJoins;$('turnCount').textContent=D.summary.tightTurnMarkers;new ResizeObserver(resize).observe(C);requestAnimationFrame(()=>fit());
</script></body></html>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--edits", type=Path, default=DEFAULT_EDITS)
    args = parser.parse_args()
    payload = build_payload(args.edits)
    OUTDIR.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    page = TEMPLATE.replace("__DATA__", encoded)
    OUT.write_text(page, encoding="utf-8", newline="\n")
    SOURCE_COPY.write_bytes(args.edits.read_bytes())
    ROUTE_EXPORT.write_text(json.dumps({
        "status": "REVIEW_ONLY_NOT_DRIVEABLE",
        "source": payload["source"],
        "source_sha256": payload["sourceSha256"],
        "origin": {"lat": payload["originLat"], "lon": payload["originLon"]},
        "deck_width_m": payload["deckWidthM"],
        "turn_review_radius_m": payload["turnReviewRadiusM"],
        "summary": payload["summary"],
        "route": [{
            "sequence": index + 1,
            "east_m": entry["p"][0],
            "north_m": entry["p"][1],
            "distance_m": entry["d"],
            "source_distance_m": entry["sourceD"],
            "segment_id": payload["segments"][entry["segment"]]["id"],
            "segment_label": payload["segments"][entry["segment"]]["label"],
            "kind": entry["kind"],
        } for index, entry in enumerate(payload["route"])],
        "remaining_joins": payload["joins"],
        "owner_approved_splices": payload["approvedSplices"],
    }, indent=2), encoding="utf-8", newline="\n")
    REPORT.write_text(json.dumps({
        "status": "REVIEW_ONLY_NOT_DRIVEABLE",
        "source_file": str(args.edits),
        "source_sha256": payload["sourceSha256"],
        "summary": payload["summary"],
        "omitted_marks": payload["omitted"],
        "owner_approved_splices": payload["approvedSplices"],
        "notes": [
            "Accepted original segments and cleaned submitted paths are ordered by their nearest original mission anchor.",
            "The September 29 owner-directed shortcuts and front/over-road sequence change are applied as solid green path geometry.",
            "The third review clips four more diversions, reverses the cleaned access path for the return, and finishes via P21 plus A15 to parking.",
            "The final review clips the 782.0–782.5 m and 787.3–789.0 m parking-return diversions.",
            "Source-distance references remain embedded so the first review's distance values can still be identified after route length changes.",
            "Red dashed joins are not approved connectors.",
            "Turn markers are review flags, not a certified vehicle-envelope analysis.",
        ],
    }, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps({"html": str(OUT), "report": str(REPORT), "route": str(ROUTE_EXPORT), "source_copy": str(SOURCE_COPY), **payload["summary"], "omitted": payload["omitted"]}, indent=2))


if __name__ == "__main__":
    main()
