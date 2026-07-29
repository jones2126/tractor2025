#!/usr/bin/env python3
"""Shared geometry and file helpers for the interactive site mission tools.

The local projection intentionally matches ``PurePursuit.latlon_to_xy`` in
``pure_pursuit_controller_20260714.py``.  Keeping the same projection avoids a
small but needless mismatch between the planner and the live controller.

This module uses only the Python standard library.  Scripts that need polygon
operations or plots import Shapely and Matplotlib themselves and provide a
clear installation message if either dependency is missing.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence


M_PER_DEG_LAT = 110_540.0
M_PER_DEG_LON_EQUATOR = 111_320.0
TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class LocalFrame:
    """Small-site east/north frame using the controller's projection."""

    ref_lat: float
    ref_lon: float

    @property
    def meters_per_degree_lon(self) -> float:
        return M_PER_DEG_LON_EQUATOR * math.cos(math.radians(self.ref_lat))

    def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        east = (lon - self.ref_lon) * self.meters_per_degree_lon
        north = (lat - self.ref_lat) * M_PER_DEG_LAT
        return east, north

    def to_latlon(self, east: float, north: float) -> tuple[float, float]:
        lat = self.ref_lat + north / M_PER_DEG_LAT
        lon = self.ref_lon + east / self.meters_per_degree_lon
        return lat, lon

    def to_dict(self) -> dict[str, float]:
        return {"ref_lat": self.ref_lat, "ref_lon": self.ref_lon}

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "LocalFrame":
        return cls(float(data["ref_lat"]), float(data["ref_lon"]))


@dataclass(frozen=True)
class Obstacle:
    name: str
    lat: float
    lon: float
    radius_m: float
    safety_margin_m: float

    @property
    def exclusion_radius_m(self) -> float:
        return self.radius_m + self.safety_margin_m


@dataclass(frozen=True)
class MissionPoint:
    lat: float
    lon: float
    yaw_rad: float
    lookahead_m: float
    speed_mps: float


def require_shapely():
    try:
        import shapely  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "Shapely is required for polygon planning.\n"
            "Install the planner dependencies with:\n"
            "  python3 -m pip install -r requirements_site_planner_20260724.txt"
        ) from exc


def require_matplotlib():
    try:
        import matplotlib  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "Matplotlib is required for checkpoint plots.\n"
            "Install the planner dependencies with:\n"
            "  python3 -m pip install -r requirements_site_planner_20260724.txt"
        ) from exc


def read_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open("r", newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def write_csv(
    path: str | Path,
    fieldnames: Sequence[str],
    rows: Iterable[dict[str, object]],
) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def read_json(path: str | Path) -> dict[str, object]:
    with Path(path).open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def write_json(path: str | Path, value: dict[str, object]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def is_included(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "t", "yes", "y", "keep"}


def optional_float(value: object) -> float | None:
    text = str(value).strip()
    if not text:
        return None
    try:
        parsed = float(text)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def polyline_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def signed_area(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    total = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        total += x1 * y2 - x2 * y1
    return total / 2.0


def normalize_angle(angle: float) -> float:
    return angle % TWO_PI


def signed_angle_delta(a: float, b: float) -> float:
    """Smallest signed rotation from angle ``a`` to angle ``b``."""

    return (b - a + math.pi) % TWO_PI - math.pi


def segment_yaw(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def densify_polyline(
    points: Sequence[tuple[float, float]],
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Sample a polyline with no gap larger than ``spacing_m``."""

    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    if not points:
        return []
    dense = [points[0]]
    for start, end in zip(points, points[1:]):
        length = distance(start, end)
        if length <= 1e-9:
            continue
        steps = max(1, math.ceil(length / spacing_m))
        for index in range(1, steps + 1):
            fraction = index / steps
            dense.append(
                (
                    start[0] + (end[0] - start[0]) * fraction,
                    start[1] + (end[1] - start[1]) * fraction,
                )
            )
    return dense


def deduplicate_points(
    points: Sequence[tuple[float, float]],
    tolerance_m: float = 1e-4,
) -> list[tuple[float, float]]:
    if not points:
        return []
    kept = [points[0]]
    for point in points[1:]:
        if distance(kept[-1], point) > tolerance_m:
            kept.append(point)
    return kept


def rotate_closed_ring(
    points: Sequence[tuple[float, float]],
    anchor: tuple[float, float],
    clockwise: bool,
) -> list[tuple[float, float]]:
    """Orient and rotate a closed ring so it starts nearest ``anchor``."""

    ring = list(points)
    if len(ring) > 1 and distance(ring[0], ring[-1]) < 1e-6:
        ring.pop()
    if len(ring) < 3:
        raise ValueError("A ring needs at least three distinct points")

    currently_clockwise = signed_area(ring) < 0.0
    if currently_clockwise != clockwise:
        ring.reverse()

    start_index = min(range(len(ring)), key=lambda i: distance(ring[i], anchor))
    ring = ring[start_index:] + ring[:start_index]
    ring.append(ring[0])
    return ring


def load_boundary(
    path: str | Path,
    frame: LocalFrame | None = None,
) -> tuple[LocalFrame, list[tuple[float, float]], list[dict[str, str]]]:
    rows = read_csv(path)
    if len(rows) < 3:
        raise ValueError(f"{path} must contain at least three boundary rows")
    try:
        latlon = [(float(row["lat"]), float(row["lon"])) for row in rows]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path} must have numeric lat and lon columns") from exc
    if frame is None:
        frame = LocalFrame(*latlon[0])
    return frame, [frame.to_xy(lat, lon) for lat, lon in latlon], rows


def load_obstacles(path: str | Path | None) -> list[Obstacle]:
    if not path:
        return []
    obstacles: list[Obstacle] = []
    for row_number, row in enumerate(read_csv(path), 2):
        try:
            obstacle = Obstacle(
                name=(row.get("name") or f"obstacle_{row_number - 1}").strip(),
                lat=float(row["lat"]),
                lon=float(row["lon"]),
                radius_m=float(row["radius_m"]),
                safety_margin_m=float(row.get("safety_margin_m") or 0.0),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid obstacle row {row_number} in {path}") from exc
        if obstacle.radius_m <= 0 or obstacle.safety_margin_m < 0:
            raise ValueError(
                f"Obstacle row {row_number} needs radius_m > 0 and "
                "safety_margin_m >= 0"
            )
        obstacles.append(obstacle)
    return obstacles


def fit_conservative_circle(
    points: Sequence[tuple[float, float]],
) -> tuple[float, float, float, float]:
    """Fit an algebraic circle, then expand it to contain every logged point.

    Returns ``center_x, center_y, best_fit_radius, containing_radius``.
    The containing radius is the safe value used by the obstacle workflow.
    """

    if len(points) < 3:
        raise ValueError("At least three points are required to fit an obstacle")

    # Solve x^2 + y^2 + D*x + E*y + F = 0 by normal equations.
    sums = {
        "x": 0.0,
        "y": 0.0,
        "xx": 0.0,
        "yy": 0.0,
        "xy": 0.0,
        "z": 0.0,
        "xz": 0.0,
        "yz": 0.0,
    }
    for x, y in points:
        z = -(x * x + y * y)
        sums["x"] += x
        sums["y"] += y
        sums["xx"] += x * x
        sums["yy"] += y * y
        sums["xy"] += x * y
        sums["z"] += z
        sums["xz"] += x * z
        sums["yz"] += y * z

    matrix = [
        [sums["xx"], sums["xy"], sums["x"]],
        [sums["xy"], sums["yy"], sums["y"]],
        [sums["x"], sums["y"], float(len(points))],
    ]
    vector = [sums["xz"], sums["yz"], sums["z"]]
    try:
        d_value, e_value, f_value = solve_3x3(matrix, vector)
        center_x = -d_value / 2.0
        center_y = -e_value / 2.0
        radius_squared = center_x**2 + center_y**2 - f_value
        if radius_squared <= 0:
            raise ValueError("non-positive radius")
        best_fit_radius = math.sqrt(radius_squared)
    except ValueError:
        center_x = sum(point[0] for point in points) / len(points)
        center_y = sum(point[1] for point in points) / len(points)
        distances = [distance((center_x, center_y), point) for point in points]
        best_fit_radius = sum(distances) / len(distances)

    containing_radius = max(
        distance((center_x, center_y), point) for point in points
    )
    return center_x, center_y, best_fit_radius, containing_radius


def solve_3x3(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    augmented = [list(row) + [float(rhs)] for row, rhs in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("Singular circle-fit matrix")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return augmented[0][3], augmented[1][3], augmented[2][3]


# ---------------------------------------------------------------------------
# Dependency-free Dubins path implementation
# ---------------------------------------------------------------------------


def _dubins_word(
    mode: str,
    alpha: float,
    beta: float,
    distance_normalized: float,
) -> tuple[float, float, float] | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    d = distance_normalized

    if mode == "LSL":
        p_squared = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
        if p_squared < 0.0:
            return None
        temporary = math.atan2(cb - ca, d + sa - sb)
        return (
            normalize_angle(-alpha + temporary),
            math.sqrt(p_squared),
            normalize_angle(beta - temporary),
        )

    if mode == "RSR":
        p_squared = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
        if p_squared < 0.0:
            return None
        temporary = math.atan2(ca - cb, d - sa + sb)
        return (
            normalize_angle(alpha - temporary),
            math.sqrt(p_squared),
            normalize_angle(-beta + temporary),
        )

    if mode == "LSR":
        p_squared = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
        if p_squared < 0.0:
            return None
        straight = math.sqrt(p_squared)
        temporary = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, straight)
        return (
            normalize_angle(-alpha + temporary),
            straight,
            normalize_angle(-beta + temporary),
        )

    if mode == "RSL":
        p_squared = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
        if p_squared < 0.0:
            return None
        straight = math.sqrt(p_squared)
        temporary = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, straight)
        return (
            normalize_angle(alpha - temporary),
            straight,
            normalize_angle(beta - temporary),
        )

    if mode == "RLR":
        temporary = (
            6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)
        ) / 8.0
        if abs(temporary) > 1.0:
            return None
        middle = normalize_angle(TWO_PI - math.acos(temporary))
        first = normalize_angle(
            alpha - math.atan2(ca - cb, d - sa + sb) + middle / 2.0
        )
        return first, middle, normalize_angle(alpha - beta - first + middle)

    if mode == "LRL":
        temporary = (
            6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)
        ) / 8.0
        if abs(temporary) > 1.0:
            return None
        middle = normalize_angle(TWO_PI - math.acos(temporary))
        first = normalize_angle(
            -alpha - math.atan2(ca - cb, d + sa - sb) + middle / 2.0
        )
        return first, middle, normalize_angle(beta - alpha - first + middle)

    raise ValueError(f"Unknown Dubins mode: {mode}")


def dubins_candidates(
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    radius_m: float,
    spacing_m: float,
) -> list[dict[str, object]]:
    """Return all valid Dubins candidates, shortest first."""

    if radius_m <= 0 or spacing_m <= 0:
        raise ValueError("Dubins radius and spacing must be positive")
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    normalized_distance = math.hypot(dx, dy) / radius_m
    reference_yaw = math.atan2(dy, dx)
    alpha = normalize_angle(start[2] - reference_yaw)
    beta = normalize_angle(goal[2] - reference_yaw)

    candidates: list[dict[str, object]] = []
    for mode in ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL"):
        parameters = _dubins_word(mode, alpha, beta, normalized_distance)
        if parameters is None:
            continue
        points = _sample_dubins(start, goal, mode, parameters, radius_m, spacing_m)
        candidates.append(
            {
                "mode": mode,
                "parameters": parameters,
                "length_m": sum(parameters) * radius_m,
                "points": points,
            }
        )
    candidates.sort(key=lambda item: float(item["length_m"]))
    return candidates


def _sample_dubins(
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    mode: str,
    parameters: Sequence[float],
    radius_m: float,
    spacing_m: float,
) -> list[tuple[float, float, float]]:
    x, y, yaw = start
    sampled = [(x, y, normalize_angle(yaw))]
    for segment_type, parameter in zip(mode, parameters):
        segment_length = parameter * radius_m
        remaining = segment_length
        while remaining > 1e-9:
            step = min(spacing_m, remaining)
            if segment_type == "S":
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
            else:
                curvature = (1.0 if segment_type == "L" else -1.0) / radius_m
                new_yaw = yaw + curvature * step
                x += (math.sin(new_yaw) - math.sin(yaw)) / curvature
                y += (-math.cos(new_yaw) + math.cos(yaw)) / curvature
                yaw = new_yaw
            sampled.append((x, y, normalize_angle(yaw)))
            remaining -= step

    # Formula and integration round-off should be tiny.  Snapping the final
    # point makes file-level validation deterministic without changing the path.
    if distance((sampled[-1][0], sampled[-1][1]), (goal[0], goal[1])) < 1e-4:
        sampled[-1] = (goal[0], goal[1], normalize_angle(goal[2]))
    return sampled


def circumcircle_radius(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> float:
    side_a = distance(b, c)
    side_b = distance(a, c)
    side_c = distance(a, b)
    cross = abs(
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    )
    if cross < 1e-10 or min(side_a, side_b, side_c) < 1e-8:
        return math.inf
    # Triangle area is cross/2; circumradius = abc/(4*area).
    return side_a * side_b * side_c / (2.0 * cross)


def read_mission(path: str | Path) -> list[MissionPoint]:
    mission: list[MissionPoint] = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 5:
                raise ValueError(
                    f"{path}:{line_number}: expected five whitespace-separated values"
                )
            try:
                mission.append(MissionPoint(*map(float, parts)))
            except ValueError as exc:
                raise ValueError(
                    f"{path}:{line_number}: all five mission values must be numeric"
                ) from exc
    if not mission:
        raise ValueError(f"{path} contains no mission waypoints")
    return mission


def write_mission(path: str | Path, mission: Sequence[MissionPoint]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        for point in mission:
            handle.write(
                f"{point.lat:.9f} {point.lon:.9f} {point.yaw_rad:.6f} "
                f"{point.lookahead_m:.2f} {point.speed_mps:.2f}\n"
            )


def pairwise(values: Sequence[object]) -> Iterator[tuple[object, object]]:
    return iter(zip(values, values[1:]))
