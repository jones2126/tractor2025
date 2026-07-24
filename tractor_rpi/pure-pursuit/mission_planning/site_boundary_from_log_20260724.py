#!/usr/bin/env python3
"""Turn field-test logger CSV data into reviewed site geometry.

This is intentionally a two-checkpoint workflow:

1. ``extract`` filters and spatially thins a field logger CSV, or
   ``import-map`` reads one polygon from GeoJSON, KML, or KMZ.  Either route
   writes an Excel-editable candidate CSV and a plot.
2. Review/edit ``include``, ``sequence`` and ``notes`` in that CSV.
3. ``finalize`` validates the edited polygon, optionally simplifies GPS noise,
   and writes the boundary CSV used by the coverage planner.

The ``fit-obstacle`` subcommand uses another reviewed outline to create a
conservative circular exclusion.  It expands the fitted circle so every
included logged point is enclosed, then adds the requested safety margin.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from site_planner_common_20260724 import (
    LocalFrame,
    distance,
    fit_conservative_circle,
    is_included,
    load_obstacles,
    optional_float,
    polyline_length,
    read_csv,
    require_matplotlib,
    require_shapely,
    rotate_closed_ring,
    write_csv,
)


CANDIDATE_COLUMNS = [
    "sequence",
    "include",
    "source_row",
    "time",
    "lat",
    "lon",
    "fix_quality",
    "hdop",
    "notes",
]

BOUNDARY_COLUMNS = [
    "sequence",
    "lat",
    "lon",
    "east_m",
    "north_m",
    "source_sequence",
    "notes",
]

OBSTACLE_COLUMNS = [
    "name",
    "lat",
    "lon",
    "radius_m",
    "safety_margin_m",
    "notes",
]


def plot_setup():
    require_matplotlib()
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    return plt


def output_path(
    explicit: str | None,
    source: str,
    suffix: str,
) -> Path:
    return Path(explicit) if explicit else Path(source).with_name(
        f"{Path(source).stem}{suffix}"
    )


def parse_logged_points(args: argparse.Namespace):
    rows = read_csv(args.field_log)
    accepted: list[dict[str, object]] = []
    rejected = {
        "row_range": 0,
        "missing_position": 0,
        "fix_quality": 0,
        "hdop": 0,
    }

    for data_row, row in enumerate(rows, 1):
        if data_row < args.start_row or (
            args.end_row is not None and data_row > args.end_row
        ):
            rejected["row_range"] += 1
            continue

        lat = optional_float(row.get("lat"))
        lon = optional_float(row.get("lon"))
        if lat is None or lon is None or not (-90 <= lat <= 90 and -180 <= lon <= 180):
            rejected["missing_position"] += 1
            continue

        quality = str(row.get("fix_quality", "")).strip()
        if args.fix_quality and quality.lower() != args.fix_quality.lower():
            rejected["fix_quality"] += 1
            continue

        hdop = optional_float(row.get("hdop"))
        if args.max_hdop is not None and (hdop is None or hdop > args.max_hdop):
            rejected["hdop"] += 1
            continue

        accepted.append(
            {
                "source_row": data_row + 1,  # Header is Excel/CSV row 1.
                "time": row.get("time", ""),
                "lat": lat,
                "lon": lon,
                "fix_quality": quality,
                "hdop": "" if hdop is None else hdop,
            }
        )

    if len(accepted) < 3:
        raise ValueError(
            "Fewer than three usable GPS points remain. Check row range, "
            "fix-quality and HDOP filters."
        )
    return accepted, rejected


def run_extract(args: argparse.Namespace) -> int:
    accepted, rejected = parse_logged_points(args)
    frame = LocalFrame(float(accepted[0]["lat"]), float(accepted[0]["lon"]))

    all_xy = [
        frame.to_xy(float(row["lat"]), float(row["lon"])) for row in accepted
    ]
    retained_indices = [0]
    last_kept = all_xy[0]
    for index, xy in enumerate(all_xy[1:], 1):
        if distance(last_kept, xy) >= args.min_spacing_m:
            retained_indices.append(index)
            last_kept = xy
    if retained_indices[-1] != len(all_xy) - 1:
        retained_indices.append(len(all_xy) - 1)

    candidates = []
    for sequence, index in enumerate(retained_indices, 1):
        row = accepted[index]
        candidates.append(
            {
                "sequence": sequence,
                "include": "y",
                "source_row": row["source_row"],
                "time": row["time"],
                "lat": f"{float(row['lat']):.9f}",
                "lon": f"{float(row['lon']):.9f}",
                "fix_quality": row["fix_quality"],
                "hdop": row["hdop"],
                "notes": "",
            }
        )

    csv_path = output_path(args.output, args.field_log, "_boundary_candidates.csv")
    plot_path = output_path(args.plot, args.field_log, "_boundary_candidates.png")
    write_csv(csv_path, CANDIDATE_COLUMNS, candidates)

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(
        [point[0] for point in all_xy],
        [point[1] for point in all_xy],
        color="0.75",
        linewidth=1,
        label=f"Filtered log ({len(all_xy)} points)",
    )
    retained_xy = [all_xy[index] for index in retained_indices]
    ax.plot(
        [point[0] for point in retained_xy],
        [point[1] for point in retained_xy],
        "o-",
        color="#1565c0",
        markersize=3,
        linewidth=1.3,
        label=f"Candidate boundary ({len(retained_xy)} points)",
    )
    ax.scatter(*retained_xy[0], s=80, color="#2e7d32", label="Start")
    ax.scatter(*retained_xy[-1], s=80, marker="x", color="#c62828", label="End")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East of first accepted fix (m)")
    ax.set_ylabel("North of first accepted fix (m)")
    ax.set_title("Boundary extraction checkpoint")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    print(f"Accepted GPS rows : {len(accepted)}")
    print(f"Candidate points  : {len(candidates)}")
    print(f"Spatial spacing   : {args.min_spacing_m:.2f} m minimum")
    print(f"Rejected rows     : {rejected}")
    print(f"Candidate CSV     : {csv_path}")
    print(f"Checkpoint plot   : {plot_path}")
    print(
        "\nNext: review include/sequence/notes in the candidate CSV, then run "
        "the finalize subcommand."
    )
    return 0


def map_polygon_latlon(path: str) -> list[tuple[float, float]]:
    """Read exactly one exterior polygon ring from GeoJSON, KML, or KMZ."""
    source = Path(path)
    suffix = source.suffix.lower()

    if suffix in {".geojson", ".json"}:
        with source.open("r", encoding="utf-8-sig") as handle:
            document = json.load(handle)
        if not isinstance(document, dict):
            raise ValueError("GeoJSON root must be an object")
        geometries = []
        kind = document.get("type")
        if kind == "FeatureCollection":
            geometries = [
                feature.get("geometry")
                for feature in document.get("features", [])
                if feature.get("geometry") is not None
            ]
        elif kind == "Feature":
            geometries = [document.get("geometry")]
        else:
            geometries = [document]
        polygons = [geometry for geometry in geometries
                    if geometry and geometry.get("type") == "Polygon"]
        unsupported = [
            geometry.get("type")
            for geometry in geometries
            if geometry and geometry.get("type") != "Polygon"
        ]
        if unsupported:
            raise ValueError(
                "Map file must contain only one Polygon; found other geometry "
                f"types: {sorted(set(unsupported))}"
            )
        if len(polygons) != 1:
            raise ValueError(
                f"Expected exactly one GeoJSON Polygon; found {len(polygons)}"
            )
        rings = polygons[0].get("coordinates", [])
        if not rings:
            raise ValueError("GeoJSON Polygon has no exterior ring")
        lonlat = []
        for point in rings[0]:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                raise ValueError(f"Invalid GeoJSON coordinate: {point!r}")
            lonlat.append((float(point[0]), float(point[1])))

    elif suffix in {".kml", ".kmz"}:
        try:
            if suffix == ".kmz":
                with zipfile.ZipFile(source) as archive:
                    names = [name for name in archive.namelist()
                             if name.lower().endswith(".kml")]
                    if not names:
                        raise ValueError("KMZ archive contains no KML file")
                    xml_bytes = archive.read(names[0])
            else:
                xml_bytes = source.read_bytes()
            root = ET.fromstring(xml_bytes)
        except (ET.ParseError, zipfile.BadZipFile) as exc:
            raise ValueError(f"Could not parse {source.name}: {exc}") from exc
        coordinate_nodes = root.findall(
            ".//{*}Polygon/{*}outerBoundaryIs/{*}LinearRing/{*}coordinates"
        )
        if len(coordinate_nodes) != 1:
            raise ValueError(
                "Expected exactly one KML polygon exterior ring; found "
                f"{len(coordinate_nodes)}"
            )
        coordinate_text = coordinate_nodes[0].text or ""
        lonlat = []
        for token in coordinate_text.split():
            parts = token.split(",")
            if len(parts) < 2:
                raise ValueError(f"Invalid KML coordinate: {token!r}")
            lonlat.append((float(parts[0]), float(parts[1])))
    else:
        raise ValueError("Map input must end in .geojson, .json, .kml, or .kmz")

    latlon = [(lat, lon) for lon, lat in lonlat]
    if len(latlon) > 3 and latlon[0] == latlon[-1]:
        latlon.pop()
    if len(latlon) < 3:
        raise ValueError("Imported polygon needs at least three distinct vertices")
    for lat, lon in latlon:
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"Invalid imported latitude/longitude: {lat}, {lon}")
    return latlon


def run_import_map(args: argparse.Namespace) -> int:
    latlon = map_polygon_latlon(args.map_file)
    frame = LocalFrame(*latlon[0])
    xy = [frame.to_xy(lat, lon) for lat, lon in latlon]
    candidates = []
    for sequence, (lat, lon) in enumerate(latlon, 1):
        candidates.append(
            {
                "sequence": sequence,
                "include": "y",
                "source_row": "",
                "time": "",
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "fix_quality": "map digitized",
                "hdop": "",
                "notes": f"Imported from {Path(args.map_file).name}",
            }
        )

    csv_path = output_path(args.output, args.map_file, "_boundary_candidates.csv")
    plot_path = output_path(args.plot, args.map_file, "_boundary_candidates.png")
    write_csv(csv_path, CANDIDATE_COLUMNS, candidates)

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(10, 8))
    closed = xy + xy[:1]
    ax.fill(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color="#90caf9",
        alpha=0.25,
    )
    ax.plot(
        [point[0] for point in closed],
        [point[1] for point in closed],
        "o-",
        color="#0d47a1",
        markersize=5,
        linewidth=1.4,
        label=f"Imported polygon ({len(xy)} vertices)",
    )
    for index, point in enumerate(xy, 1):
        ax.annotate(str(index), point, fontsize=8, xytext=(4, 4),
                    textcoords="offset points")
    ax.scatter(*xy[0], s=90, color="#2e7d32", label="Start/anchor")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East of first polygon vertex (m)")
    ax.set_ylabel("North of first polygon vertex (m)")
    ax.set_title("Map polygon import checkpoint")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    print(f"Imported vertices : {len(candidates)}")
    print(f"Candidate CSV     : {csv_path}")
    print(f"Checkpoint plot   : {plot_path}")
    print(
        "\nNext: review include/sequence/notes in the candidate CSV, then run "
        "the finalize subcommand."
    )
    return 0


def selected_candidate_rows(path: str):
    rows = read_csv(path)
    selected = [
        row
        for row in rows
        if "include" not in row or is_included(row.get("include", "y"))
    ]
    if len(selected) < 3:
        raise ValueError("At least three included rows are required")
    try:
        selected.sort(key=lambda row: float(row.get("sequence", "")))
        latlon = [(float(row["lat"]), float(row["lon"])) for row in selected]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"{path} needs numeric sequence, lat and lon values on included rows"
        ) from exc

    # A logger lap may explicitly repeat the start fix.  Polygon formats close
    # implicitly, so remove that duplicate before validation/simplification.
    if len(latlon) > 3:
        frame = LocalFrame(*latlon[0])
        if distance(frame.to_xy(*latlon[0]), frame.to_xy(*latlon[-1])) < 0.05:
            latlon.pop()
            selected.pop()
    return selected, latlon


def run_finalize(args: argparse.Namespace) -> int:
    require_shapely()
    from shapely.geometry import Polygon
    from shapely.validation import explain_validity

    selected, latlon = selected_candidate_rows(args.candidates)
    frame = LocalFrame(*latlon[0])
    xy = [frame.to_xy(lat, lon) for lat, lon in latlon]
    if args.reverse:
        xy.reverse()
        selected.reverse()

    polygon = Polygon(xy)
    if not polygon.is_valid:
        raise ValueError(
            "Boundary is not a valid simple polygon: "
            f"{explain_validity(polygon)}. Edit include/sequence and rerun."
        )
    if polygon.area < args.minimum_area_m2:
        raise ValueError(
            f"Boundary area is only {polygon.area:.1f} m^2; expected at least "
            f"{args.minimum_area_m2:.1f} m^2."
        )

    if args.simplify_m > 0:
        simplified = polygon.simplify(args.simplify_m, preserve_topology=True)
        if simplified.geom_type != "Polygon" or simplified.is_empty:
            raise ValueError("Simplification did not produce one usable polygon")
        xy_final = list(simplified.exterior.coords)[:-1]
        # Retain the operator-selected starting area after Shapely simplifies.
        xy_final = rotate_closed_ring(
            xy_final,
            anchor=xy[0],
            clockwise=False if polygon.exterior.is_ccw else True,
        )[:-1]
    else:
        xy_final = xy

    output_rows = []
    for sequence, point in enumerate(xy_final, 1):
        lat, lon = frame.to_latlon(*point)
        source_index = min(range(len(xy)), key=lambda i: distance(xy[i], point))
        output_rows.append(
            {
                "sequence": sequence,
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "east_m": f"{point[0]:.3f}",
                "north_m": f"{point[1]:.3f}",
                "source_sequence": selected[source_index].get("sequence", ""),
                "notes": selected[source_index].get("notes", ""),
            }
        )

    output = output_path(args.output, args.candidates, "_final.csv")
    plot_path = output_path(args.plot, args.candidates, "_final.png")
    write_csv(output, BOUNDARY_COLUMNS, output_rows)

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(10, 8))
    closed = xy_final + xy_final[:1]
    ax.fill(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color="#90caf9",
        alpha=0.25,
    )
    ax.plot(
        [point[0] for point in closed],
        [point[1] for point in closed],
        "o-",
        color="#0d47a1",
        markersize=3,
    )
    for index, point in enumerate(xy_final, 1):
        if len(xy_final) <= 60 or index in {1, len(xy_final)}:
            ax.annotate(str(index), point, fontsize=7, xytext=(3, 3),
                        textcoords="offset points")
    ax.scatter(*xy_final[0], s=90, color="#2e7d32", label="Mission-side anchor")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(
        f"Final boundary: {polygon.area:.1f} m², "
        f"{polygon.length:.1f} m perimeter"
    )
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    closing_edge_length = distance(xy[0], xy[-1])
    print(f"Included points   : {len(xy)}")
    print(f"Final points      : {len(xy_final)}")
    print(f"Area              : {polygon.area:.1f} m²")
    print(f"Perimeter         : {polygon.length:.1f} m")
    print(f"Closing edge length: {closing_edge_length:.2f} m")
    print(f"Boundary CSV      : {output}")
    print(f"Checkpoint plot   : {plot_path}")
    print("\nNext: use this boundary CSV with the coverage planner compare-angles step.")
    return 0


def run_fit_obstacle(args: argparse.Namespace) -> int:
    selected, latlon = selected_candidate_rows(args.candidates)
    frame = LocalFrame(*latlon[0])
    points = [frame.to_xy(lat, lon) for lat, lon in latlon]
    center_x, center_y, fitted_radius, containing_radius = fit_conservative_circle(
        points
    )
    center_lat, center_lon = frame.to_latlon(center_x, center_y)

    output = Path(args.output)
    existing = []
    if output.exists():
        # Validate before preserving existing rows.
        load_obstacles(output)
        existing = [
            row for row in read_csv(output)
            if str(row.get("name", "")).strip() != args.name
        ]
    existing.append(
        {
            "name": args.name,
            "lat": f"{center_lat:.9f}",
            "lon": f"{center_lon:.9f}",
            "radius_m": f"{containing_radius:.3f}",
            "safety_margin_m": f"{args.safety_margin_m:.3f}",
            "notes": (
                f"Conservative circle from {len(points)} included outline points; "
                f"least-squares radius={fitted_radius:.3f} m"
            ),
        }
    )
    write_csv(output, OBSTACLE_COLUMNS, existing)

    plot_path = output_path(args.plot, args.candidates, f"_{args.name}_circle.png")
    plt = plot_setup()
    from matplotlib.patches import Circle

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(
        [point[0] for point in points],
        [point[1] for point in points],
        color="#c62828",
        s=18,
        label="Included outline fixes",
    )
    ax.add_patch(
        Circle(
            (center_x, center_y),
            fitted_radius,
            fill=False,
            color="#1565c0",
            linestyle="--",
            label="Least-squares fit",
        )
    )
    ax.add_patch(
        Circle(
            (center_x, center_y),
            containing_radius,
            fill=False,
            color="#ef6c00",
            label="Radius containing every fix",
        )
    )
    ax.add_patch(
        Circle(
            (center_x, center_y),
            containing_radius + args.safety_margin_m,
            color="#ef6c00",
            alpha=0.15,
            label="Planner exclusion with margin",
        )
    )
    ax.scatter(center_x, center_y, marker="+", s=90, color="black")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"Obstacle checkpoint: {args.name}")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    print(f"Included points       : {len(points)}")
    print(f"Least-squares radius  : {fitted_radius:.2f} m")
    print(f"Containing radius     : {containing_radius:.2f} m")
    print(f"Safety margin         : {args.safety_margin_m:.2f} m")
    print(f"Total exclusion radius: {containing_radius + args.safety_margin_m:.2f} m")
    print(f"Obstacle CSV          : {output}")
    print(f"Checkpoint plot       : {plot_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare reviewed site boundaries/obstacles from field logs or "
            "map polygons"
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    extract = subparsers.add_parser(
        "extract",
        help="filter and spatially thin a field_test_logger CSV",
    )
    extract.add_argument("field_log")
    extract.add_argument("--output", help="candidate CSV output path")
    extract.add_argument("--plot", help="checkpoint PNG output path")
    extract.add_argument(
        "--fix-quality",
        default="RTK Fixed",
        help="exact fix quality to retain; use an empty string for any quality",
    )
    extract.add_argument(
        "--max-hdop",
        type=float,
        default=None,
        help="optional maximum HDOP; rows with blank HDOP are rejected",
    )
    extract.add_argument(
        "--start-row",
        type=int,
        default=1,
        help="first CSV data row after the header (default 1)",
    )
    extract.add_argument(
        "--end-row",
        type=int,
        default=None,
        help="last CSV data row after the header (default end of file)",
    )
    extract.add_argument(
        "--min-spacing-m",
        type=float,
        default=0.50,
        help="minimum distance between candidate points (default 0.50 m)",
    )
    extract.set_defaults(handler=run_extract)

    import_map = subparsers.add_parser(
        "import-map",
        help="import one polygon from GeoJSON, KML, or KMZ",
    )
    import_map.add_argument("map_file")
    import_map.add_argument("--output", help="candidate CSV output path")
    import_map.add_argument("--plot", help="checkpoint PNG output path")
    import_map.set_defaults(handler=run_import_map)

    finalize = subparsers.add_parser(
        "finalize",
        help="validate an edited candidate CSV and create the planner boundary",
    )
    finalize.add_argument("candidates")
    finalize.add_argument("--output", help="final boundary CSV output path")
    finalize.add_argument("--plot", help="checkpoint PNG output path")
    finalize.add_argument(
        "--simplify-m",
        type=float,
        default=0.10,
        help="topology-preserving simplification tolerance (default 0.10 m)",
    )
    finalize.add_argument(
        "--minimum-area-m2",
        type=float,
        default=25.0,
        help="reject suspiciously small polygons (default 25 m²)",
    )
    finalize.add_argument(
        "--reverse",
        action="store_true",
        help="reverse the reviewed traversal order",
    )
    finalize.set_defaults(handler=run_finalize)

    obstacle = subparsers.add_parser(
        "fit-obstacle",
        help="fit a conservative circular exclusion to reviewed outline points",
    )
    obstacle.add_argument("candidates")
    obstacle.add_argument("--name", required=True)
    obstacle.add_argument("--output", required=True, help="obstacles CSV to create/update")
    obstacle.add_argument("--plot", help="checkpoint PNG output path")
    obstacle.add_argument(
        "--safety-margin-m",
        type=float,
        default=0.75,
        help="extra radius beyond every logged outline point (default 0.75 m)",
    )
    obstacle.set_defaults(handler=run_fit_obstacle)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if getattr(args, "min_spacing_m", 1.0) <= 0:
        parser.error("--min-spacing-m must be positive")
    if getattr(args, "start_row", 1) < 1:
        parser.error("--start-row must be at least 1")
    if getattr(args, "simplify_m", 0.0) < 0:
        parser.error("--simplify-m cannot be negative")
    if getattr(args, "safety_margin_m", 0.0) < 0:
        parser.error("--safety-margin-m cannot be negative")
    try:
        return int(args.handler(args))
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
