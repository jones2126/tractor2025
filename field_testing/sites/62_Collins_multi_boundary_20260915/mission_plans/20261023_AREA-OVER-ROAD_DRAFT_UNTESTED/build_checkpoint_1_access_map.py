#!/usr/bin/env python3
"""Build the Checkpoint 1 access-evidence map for AREA-OVER-ROAD."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch
from shapely.geometry import shape
from shapely.ops import transform


STATUS = "DRAFT - UNTESTED IN THE FIELD"
ROOT = Path(__file__).resolve().parents[5]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
OUTPUT = Path(__file__).with_name("checkpoint_1_access_map_rev001.png")


def main() -> None:
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    mowable_data = json.loads(MOWABLE.read_text(encoding="utf-8"))
    mowable = shape(mowable_data["features"][0]["geometry"])

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

    by_id = {}
    for feature in inventory["features"]:
        by_id.setdefault(feature["properties"].get("asset_id"), []).append(feature)

    over = project(mowable)
    front = project(shape(by_id["AREA-FRONT-YARD"][0]["geometry"]))
    route_out = project(shape(by_id["ROUTE-FRONT-TO-OVERROAD"][0]["geometry"]))
    route_in = project(shape(by_id["ROUTE-OVERROAD-TO-FRONT"][0]["geometry"]))

    access_ids = [
        "ACCESS-FRONT-NORTH-OUTBOUND",
        "ACCESS-OVERROAD-NORTH-OUTBOUND",
        "ACCESS-OVERROAD-NORTH-INBOUND",
        "ACCESS-FRONT-NORTH-INBOUND",
    ]
    access = {
        asset_id: project(shape(by_id[asset_id][0]["geometry"]))
        for asset_id in access_ids
    }

    fig, ax = plt.subplots(figsize=(9.5, 7.0), dpi=180)

    def add_polygon(poly, face, edge, label, alpha=0.25):
        ax.add_patch(
            PolygonPatch(
                list(poly.exterior.coords),
                closed=True,
                facecolor=face,
                edgecolor=edge,
                linewidth=2.0,
                alpha=alpha,
                label=label,
            )
        )
        for hole in poly.interiors:
            ax.add_patch(
                PolygonPatch(
                    list(hole.coords),
                    closed=True,
                    facecolor="white",
                    edgecolor="#b91c1c",
                    linewidth=2.0,
                    hatch="///",
                    label="OBS-POLE-001 exclusion",
                )
            )

    add_polygon(front, "#94a3b8", "#475569", "Front yard (context)", 0.20)
    add_polygon(over, "#86efac", "#15803d", "AREA-OVER-ROAD", 0.40)

    def plot_route(line, color, label):
        x, y = line.xy
        ax.plot(x, y, color=color, linewidth=3.0, label=label, zorder=4)
        mid = len(x) // 2
        start = max(0, mid - 2)
        end = min(len(x) - 1, mid + 2)
        ax.add_patch(
            FancyArrowPatch(
                (x[start], y[start]),
                (x[end], y[end]),
                arrowstyle="-|>",
                mutation_scale=16,
                linewidth=2.0,
                color=color,
                zorder=5,
            )
        )

    plot_route(route_out, "#2563eb", "Recorded outbound: front to over-road")
    plot_route(route_in, "#d97706", "Recorded return: over-road to front")

    marker_style = {
        "ACCESS-FRONT-NORTH-OUTBOUND": ("#2563eb", "F-out"),
        "ACCESS-OVERROAD-NORTH-OUTBOUND": ("#2563eb", "O-in"),
        "ACCESS-OVERROAD-NORTH-INBOUND": ("#d97706", "O-out"),
        "ACCESS-FRONT-NORTH-INBOUND": ("#d97706", "F-in"),
    }
    offsets = {
        "ACCESS-FRONT-NORTH-OUTBOUND": (-18, -14),
        "ACCESS-OVERROAD-NORTH-OUTBOUND": (7, 11),
        "ACCESS-OVERROAD-NORTH-INBOUND": (7, -18),
        "ACCESS-FRONT-NORTH-INBOUND": (-26, 8),
    }
    for asset_id, point in access.items():
        color, short = marker_style[asset_id]
        ax.scatter(point.x, point.y, s=62, color=color, edgecolor="white", linewidth=1.3, zorder=6)
        ax.annotate(
            short,
            (point.x, point.y),
            xytext=offsets[asset_id],
            textcoords="offset points",
            fontsize=9,
            fontweight="bold",
            color=color,
            bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": color, "alpha": 0.9},
            zorder=7,
        )

    ax.annotate(
        "Named over-road points are inside the promoted boundary\n"
        "O-in: 0.49 m inside   O-out: 0.61 m inside",
        xy=(0.02, 0.02),
        xycoords="axes fraction",
        fontsize=9,
        color="#7c2d12",
        bbox={"boxstyle": "round,pad=0.4", "fc": "#fffbeb", "ec": "#f59e0b"},
    )

    ax.set_title(
        "Checkpoint 1 — AREA-OVER-ROAD access evidence",
        loc="left",
        y=1.075,
        fontsize=15,
        fontweight="bold",
    )
    ax.text(0.0, 1.01, STATUS, transform=ax.transAxes, fontsize=11, color="#b91c1c", fontweight="bold")
    ax.set_xlabel("Local east (m)")
    ax.set_ylabel("Local north (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#e2e8f0", linewidth=0.7)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="upper left", fontsize=8, framealpha=0.95)
    fig.text(
        0.99,
        0.01,
        "Recorded routes are approved existing evidence in recorded direction only; reverse travel is unvalidated.",
        ha="right",
        va="bottom",
        fontsize=8,
        color="#334155",
    )
    fig.tight_layout(rect=(0, 0.035, 1, 0.94))
    fig.savefig(OUTPUT, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
