#!/usr/bin/env python3
"""Build revised Checkpoint 2 map with the progressive obstacle bypass."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch
from shapely.geometry import Polygon, shape
from shapely.ops import transform


STATUS = "DRAFT - UNTESTED IN THE FIELD"
ROOT = Path(__file__).resolve().parents[5]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
OUTPUT = Path(__file__).with_name("checkpoint_2_necessary_decomposition_rev002.png")


def main() -> None:
    geographic = shape(json.loads(MOWABLE.read_text(encoding="utf-8"))["features"][0]["geometry"])
    ref_lat = geographic.centroid.y
    ref_lon = geographic.centroid.x
    east_scale = 111_320.0 * math.cos(math.radians(ref_lat))
    north_scale = 110_540.0
    local = transform(
        lambda lon, lat, z=None: ((lon - ref_lon) * east_scale, (lat - ref_lat) * north_scale),
        geographic,
    )
    outer = Polygon(local.exterior)
    hole = Polygon(local.interiors[0])
    coords = np.asarray(local.exterior.coords)[:-1]
    values, vectors = np.linalg.eigh(np.cov(coords.T))
    long_axis = vectors[:, np.argmax(values)]
    if long_axis[1] < 0:
        long_axis = -long_axis
    cross_axis = np.array([-long_axis[1], long_axis[0]])

    fig, ax = plt.subplots(figsize=(8.2, 9.0), dpi=180)
    ax.add_patch(
        PolygonPatch(
            list(outer.exterior.coords),
            closed=True,
            facecolor="#dcfce7",
            edgecolor="#15803d",
            linewidth=2.4,
            alpha=0.9,
            label="AREA-OVER-ROAD — one mowing area",
        )
    )
    ax.add_patch(
        PolygonPatch(
            list(hole.exterior.coords),
            closed=True,
            facecolor="white",
            edgecolor="#b91c1c",
            linewidth=2.2,
            hatch="///",
            label="OBS-POLE-001 exclusion",
        )
    )

    center = np.array([hole.centroid.x, hole.centroid.y])
    theta = np.linspace(0.0, 2.0 * math.pi, 160)
    loop = np.column_stack((center[0] + 1.4 * np.cos(theta), center[1] + 1.4 * np.sin(theta)))
    ax.plot(loop[:, 0], loop[:, 1], color="#ea580c", linewidth=2.8, label="1.4 m perimeter obstacle loop concept")
    for index in (35, 105):
        ax.add_patch(
            FancyArrowPatch(
                tuple(loop[index]),
                tuple(loop[index + 7]),
                arrowstyle="-|>",
                mutation_scale=14,
                linewidth=2.0,
                color="#ea580c",
                zorder=7,
            )
        )

    # Conceptual stripe continuation and local detour, intentionally not mission geometry.
    start = center - long_axis * 13.0
    end = center + long_axis * 13.0
    detour_a = center - long_axis * 3.0 - cross_axis * 0.3
    detour_b = center - cross_axis * 1.7
    detour_c = center + long_axis * 3.0 - cross_axis * 0.3
    stripe = np.vstack((start, detour_a, detour_b, detour_c, end))
    ax.plot(
        stripe[:, 0],
        stripe[:, 1],
        color="#2563eb",
        linewidth=2.6,
        linestyle="--",
        label="Stripe-preserving obstacle bypass concept",
    )
    ax.add_patch(
        FancyArrowPatch(
            tuple(stripe[1]),
            tuple(stripe[3]),
            connectionstyle="arc3,rad=-0.28",
            arrowstyle="-|>",
            mutation_scale=14,
            linewidth=2.2,
            color="#2563eb",
            zorder=8,
        )
    )

    min_long = float(np.min(coords @ long_axis))
    max_long = float(np.max(coords @ long_axis))
    middle_cross = float(np.mean(coords @ cross_axis))
    for value, label in [(max_long - 4.5, "North turn zone"), (min_long + 4.5, "South turn zone")]:
        point = long_axis * value + cross_axis * middle_cross
        ax.annotate(
            label,
            tuple(point),
            ha="center",
            fontsize=9,
            fontweight="bold",
            color="#7c3aed",
            bbox={"boxstyle": "round,pad=0.3", "fc": "#f5f3ff", "ec": "#8b5cf6"},
        )

    ax.annotate(
        "Progressive obstacle bypass\n1. perimeter loop and edge rejoin\n2. later stripes detour and resume",
        (hole.centroid.x, hole.centroid.y),
        xytext=(13.0, 5.0),
        textcoords="data",
        fontsize=9,
        color="#7c2d12",
        bbox={"boxstyle": "round,pad=0.4", "fc": "#fff7ed", "ec": "#ea580c"},
        arrowprops={"arrowstyle": "->", "color": "#ea580c"},
    )
    ax.annotate(
        "Complete this entire mowing area\nbefore using the exit path",
        (local.centroid.x + 1.0, local.centroid.y - 16.0),
        ha="center",
        fontsize=11,
        fontweight="bold",
        color="#14532d",
        bbox={"boxstyle": "round,pad=0.4", "fc": "white", "ec": "#15803d"},
    )

    ax.set_title("Checkpoint 2 — revised necessary decomposition", loc="left", y=1.065, fontsize=15, fontweight="bold")
    ax.text(0.0, 1.012, STATUS, transform=ax.transAxes, fontsize=11, color="#b91c1c", fontweight="bold")
    ax.set_xlabel("Local east (m)")
    ax.set_ylabel("Local north (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#e2e8f0", linewidth=0.7)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="lower left", fontsize=8, framealpha=0.95)
    fig.text(0.99, 0.01, "Planning concept only — no stripes, waypoints, or executable mission.", ha="right", fontsize=8, color="#334155")
    fig.tight_layout(rect=(0, 0.035, 1, 0.95))
    fig.savefig(OUTPUT, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
