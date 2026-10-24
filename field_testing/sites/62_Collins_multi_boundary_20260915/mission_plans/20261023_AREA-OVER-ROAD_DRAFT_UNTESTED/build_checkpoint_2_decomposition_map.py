#!/usr/bin/env python3
"""Build the Checkpoint 2 necessary-decomposition map for AREA-OVER-ROAD."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch
from shapely.geometry import LineString, Polygon, shape
from shapely.ops import transform


STATUS = "DRAFT - UNTESTED IN THE FIELD"
CUT_WIDTH_M = 0.9652
TURN_RADIUS_M = 1.4
ROOT = Path(__file__).resolve().parents[5]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
OUTPUT = Path(__file__).with_name("checkpoint_2_necessary_decomposition_rev001.png")


def add_polygon(ax, polygon, **kwargs):
    ax.add_patch(PolygonPatch(list(polygon.exterior.coords), closed=True, **kwargs))


def main() -> None:
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    mowable = shape(json.loads(MOWABLE.read_text(encoding="utf-8"))["features"][0]["geometry"])
    ref_lat = mowable.centroid.y
    ref_lon = mowable.centroid.x
    east_scale = 111_320.0 * math.cos(math.radians(ref_lat))
    north_scale = 110_540.0

    def project(geometry):
        return transform(
            lambda lon, lat, z=None: (
                (lon - ref_lon) * east_scale,
                (lat - ref_lat) * north_scale,
            ),
            geometry,
        )

    local = project(mowable)
    outer = Polygon(local.exterior)
    hole = Polygon(local.interiors[0])
    centered_cut_region = local.buffer(-CUT_WIDTH_M / 2.0, join_style=2)

    coords = np.asarray(local.exterior.coords)[:-1]
    values, vectors = np.linalg.eigh(np.cov(coords.T))
    long_axis = vectors[:, np.argmax(values)]
    cross_axis = np.array([-long_axis[1], long_axis[0]])
    if long_axis[1] < 0:
        long_axis = -long_axis
        cross_axis = -cross_axis

    hole_center = np.array([hole.centroid.x, hole.centroid.y])
    station = hole_center @ long_axis
    cross_line = LineString(
        [tuple(long_axis * station + cross_axis * offset) for offset in (-100.0, 100.0)]
    )
    outer_cross = outer.intersection(cross_line)
    hole_cross = hole.intersection(cross_line)
    outer_values = sorted(np.asarray(point) @ cross_axis for point in outer_cross.coords)
    hole_values = sorted(np.asarray(point) @ cross_axis for point in hole_cross.coords)
    corridor_a = hole_values[0] - outer_values[0]
    corridor_b = outer_values[-1] - hole_values[-1]
    narrow_width = min(corridor_a, corridor_b)
    wide_width = max(corridor_a, corridor_b)

    by_id = {
        feature["properties"].get("asset_id"): feature
        for feature in inventory["features"]
        if feature["properties"].get("asset_id")
    }
    f_out = project(shape(by_id["ACCESS-FRONT-NORTH-OUTBOUND"]["geometry"]))
    o_out = project(shape(by_id["ACCESS-OVERROAD-NORTH-INBOUND"]["geometry"]))

    fig, ax = plt.subplots(figsize=(8.3, 9.2), dpi=180)
    add_polygon(
        ax,
        outer,
        facecolor="#dcfce7",
        edgecolor="#15803d",
        linewidth=2.2,
        alpha=0.85,
        label="Approved mowable polygon",
    )
    if centered_cut_region.geom_type == "Polygon":
        add_polygon(
            ax,
            centered_cut_region,
            facecolor="#bfdbfe",
            edgecolor="#2563eb",
            linewidth=1.6,
            alpha=0.65,
            label="Centered 38-in swath feasible region",
        )
    add_polygon(
        ax,
        hole,
        facecolor="white",
        edgecolor="#b91c1c",
        linewidth=2.2,
        hatch="///",
        label="OBS-POLE-001 exclusion",
    )
    ax.scatter(
        [hole.centroid.x],
        [hole.centroid.y],
        s=32,
        color="#b91c1c",
        edgecolor="white",
        linewidth=1.0,
        zorder=8,
    )

    ox, oy = outer_cross.xy
    hx, hy = hole_cross.xy
    ax.plot(ox, oy, color="#64748b", linestyle="--", linewidth=1.4, zorder=5)
    ax.plot(hx, hy, color="#b91c1c", linewidth=3.0, zorder=6)

    narrow_side = -1 if corridor_a < corridor_b else 1
    narrow_outer = np.asarray(outer_cross.coords[0 if narrow_side == -1 else -1])
    narrow_hole = np.asarray(hole_cross.coords[0 if narrow_side == -1 else -1])
    ax.add_patch(
        FancyArrowPatch(
            tuple(narrow_outer),
            tuple(narrow_hole),
            arrowstyle="<->",
            mutation_scale=13,
            linewidth=2.0,
            color="#ea580c",
            zorder=7,
        )
    )
    narrow_mid = (narrow_outer + narrow_hole) / 2.0
    ax.annotate(
        f"Narrow pole-side pocket\n{narrow_width:.3f} m < 0.965 m cut width",
        tuple(narrow_mid),
        xytext=(-30, -5),
        textcoords="data",
        fontsize=9,
        color="#9a3412",
        bbox={"boxstyle": "round,pad=0.35", "fc": "#fff7ed", "ec": "#ea580c"},
        arrowprops={"arrowstyle": "->", "color": "#ea580c"},
        zorder=8,
    )

    ax.annotate(
        f"Wide side remains connected\n{wide_width:.3f} m at pole station",
        (hole.centroid.x, hole.centroid.y),
        xytext=(58, 34),
        textcoords="offset points",
        fontsize=9,
        color="#1d4ed8",
        bbox={"boxstyle": "round,pad=0.35", "fc": "#eff6ff", "ec": "#2563eb"},
        arrowprops={"arrowstyle": "->", "color": "#2563eb"},
        zorder=8,
    )

    min_long = float(np.min(coords @ long_axis))
    max_long = float(np.max(coords @ long_axis))
    for value, label in [
        (max_long - 4.5, "North turn zone"),
        (min_long + 4.5, "South turn zone"),
    ]:
        center = long_axis * value + cross_axis * float(np.mean(coords @ cross_axis))
        ax.annotate(
            label,
            tuple(center),
            ha="center",
            va="center",
            fontsize=9,
            fontweight="bold",
            color="#7c3aed",
            bbox={"boxstyle": "round,pad=0.3", "fc": "#f5f3ff", "ec": "#8b5cf6"},
        )

    ax.annotate(
        "CELL-OR-MAIN\none connected coverage cell",
        (local.centroid.x + 1.0, local.centroid.y - 14.0),
        ha="center",
        fontsize=11,
        fontweight="bold",
        color="#14532d",
        bbox={"boxstyle": "round,pad=0.4", "fc": "white", "ec": "#15803d", "alpha": 0.9},
    )

    access = LineString([(f_out.x, f_out.y), (o_out.x, o_out.y)])
    ax.plot(*access.xy, color="#7c3aed", linewidth=3.0, linestyle=":", label="Approved F-out ↔ O-out access assumption")
    ax.add_patch(
        FancyArrowPatch(
            (f_out.x, f_out.y),
            (o_out.x, o_out.y),
            arrowstyle="<|-|>",
            mutation_scale=15,
            linewidth=1.8,
            color="#7c3aed",
            zorder=7,
        )
    )
    ax.scatter([f_out.x, o_out.x], [f_out.y, o_out.y], s=48, color="#7c3aed", edgecolor="white", zorder=8)

    ax.set_title(
        "Checkpoint 2 — necessary decomposition",
        loc="left",
        y=1.065,
        fontsize=15,
        fontweight="bold",
    )
    ax.text(0.0, 1.012, STATUS, transform=ax.transAxes, fontsize=11, color="#b91c1c", fontweight="bold")
    ax.set_xlabel("Local east (m)")
    ax.set_ylabel("Local north (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#e2e8f0", linewidth=0.7)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="lower left", fontsize=8, framealpha=0.95)
    fig.text(
        0.99,
        0.01,
        f"Planning evidence only: 38-in cut width; {TURN_RADIUS_M:.1f} m test radius; no stripes or executable mission.",
        ha="right",
        fontsize=8,
        color="#334155",
    )
    fig.tight_layout(rect=(0, 0.035, 1, 0.95))
    fig.savefig(OUTPUT, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
