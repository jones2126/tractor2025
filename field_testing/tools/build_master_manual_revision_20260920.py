#!/usr/bin/env python3
"""Build a REVIEW-ONLY master-route revision from original W and recorded M IDs.

This deliberately does not create a launcher or change the archived mission.
All W references are one-based indices in the 19,250-point archived mission.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from build_master_manual_overlay_20260919 import FIELD_LOG, MANUAL_END, MANUAL_START, period
from build_master_target_mission_viewer_20260919 import AUDIT, MISSION, SITE, load


OUT = SITE / "analysis" / "master_manual_revision_v2_REVIEW_ONLY_20260920"
SPEED_MPS = 1.0  # Consistent with the user's prior all-1.0-m/s revision request.
LOOKAHEAD_M = 2.0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    return ((lon - lon0) * 111_320 * math.cos(math.radians(lat0)),
            (lat - lat0) * 111_132)


def ll(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
    return lat0 + y / 111_132, lon0 + x / (111_320 * math.cos(math.radians(lat0)))


def bearing_to_yaw(heading: float) -> float:
    return math.radians(90 - heading)


def sample_manual(lat0: float, lon0: float) -> list[dict]:
    points = []
    previous = None
    with FIELD_LOG.open(newline="", encoding="utf-8-sig") as handle:
        for field_row, row in enumerate(csv.DictReader(handle), 1):
            stamp = row.get("time", "")
            if not MANUAL_START <= stamp < MANUAL_END or row.get("steer_mode") != "1":
                continue
            lat, lon = float(row["lat"]), float(row["lon"])
            if not (math.isfinite(lat) and math.isfinite(lon)):
                continue
            group = period(stamp)
            repeated = previous is not None and (lat, lon) == previous[:2]
            if repeated and group == previous[2]:
                continue
            previous = (lat, lon, group)
            points.append(dict(id=f"M{len(points)+1}", lat=lat, lon=lon,
                               xy=xy(lat, lon, lat0, lon0), time=stamp,
                               fix_quality=row["fix_quality"],
                               heading=float(row["heading_deg"]),
                               field_row=field_row, group=group))
    if len(points) != 3600 or points[17]["time"] != "2026-09-18T16:56:16.440+00:00":
        raise ValueError("Manual M numbering differs from approved overlay")
    return points


def lerp_connector(a: dict, b: dict, prefix: str, lat0: float, lon0: float,
                   spacing: float = 0.25) -> list[dict]:
    ax, ay = a["xy"]
    bx, by = b["xy"]
    length = math.hypot(bx - ax, by - ay)
    heading = (90 - math.degrees(math.atan2(by - ay, bx - ax))) % 360
    count = max(1, math.ceil(length / spacing))
    out = []
    for i in range(1, count):
        t = i / count
        x, y = ax + t * (bx - ax), ay + t * (by - ay)
        lat, lon = ll(x, y, lat0, lon0)
        out.append(dict(id=f"{prefix}{i}", lat=lat, lon=lon, xy=(x, y),
                        heading=heading, phase=f"review_connector_{prefix}"))
    return out


def bezier_return(a: dict, b: dict, lat0: float, lon0: float) -> list[dict]:
    """A direct, gently curved 21-m return matching recorded/start headings."""
    p0, p3 = a["xy"], b["xy"]
    distance = math.dist(p0, p3)
    handle = min(4.0, distance / 4)
    def direction(heading: float):
        h = math.radians(heading)
        return math.sin(h), math.cos(h)
    u0, u1 = direction(a["heading"]), direction(b["heading"])
    p1 = (p0[0] + handle*u0[0], p0[1] + handle*u0[1])
    p2 = (p3[0] - handle*u1[0], p3[1] - handle*u1[1])
    def at(t: float):
        s = 1-t
        return (s**3*p0[0] + 3*s*s*t*p1[0] + 3*s*t*t*p2[0] + t**3*p3[0],
                s**3*p0[1] + 3*s*s*t*p1[1] + 3*s*t*t*p2[1] + t**3*p3[1])
    fine = [at(i/1000) for i in range(1001)]
    arc = sum(math.dist(x, y) for x, y in zip(fine, fine[1:]))
    count = math.ceil(arc / 0.25)
    out = []
    for i in range(1, count):
        t = i/count
        x, y = at(t)
        x2, y2 = at(min(1, t + 0.001))
        heading = (90-math.degrees(math.atan2(y2-y, x2-x))) % 360
        lat, lon = ll(x, y, lat0, lon0)
        out.append(dict(id=f"R{i}", lat=lat, lon=lon, xy=(x, y),
                        heading=heading, phase="review_return_to_W12845"))
    return out


def main() -> None:
    payload = load()  # Checks archive count, coordinate agreement, and provenance.
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        source_audit = list(csv.DictReader(handle))
    with MISSION.open(encoding="utf-8") as handle:
        source_mission = [line.split() for line in handle if line.strip()]
    lat0, lon0 = payload["points"][0][2:4]
    master = []
    for i, (values, row) in enumerate(zip(source_mission, source_audit), 1):
        lat, lon, yaw, lookahead, speed = map(float, values)
        master.append(dict(id=f"W{i}", lat=lat, lon=lon, xy=xy(lat, lon, lat0, lon0),
                           heading=(90-math.degrees(yaw)) % 360, phase=row["phase"],
                           original_speed=speed, source_waypoint=row["source_waypoint"]))
    manual = sample_manual(lat0, lon0)
    selected = manual[17:2860]
    assert selected[0]["id"] == "M18" and selected[-1]["id"] == "M2860"

    w = lambda n: master[n-1]
    entry = lerp_connector(w(12652), selected[0], "E", lat0, lon0)
    ret = bezier_return(selected[-1], w(12845), lat0, lon0)
    garden = lerp_connector(w(13642), w(13704), "G", lat0, lon0)
    exit_join = lerp_connector(w(13907), w(13941), "X", lat0, lon0)
    left_circle_1 = lerp_connector(w(14517), w(14556), "L", lat0, lon0)
    left_circle_2 = lerp_connector(w(15749), w(15807), "Q", lat0, lon0)
    left_circle_3 = lerp_connector(w(15867), w(15903), "T", lat0, lon0)
    revised = (master[:12652] + entry + selected + ret + master[12844:13642]
               + garden + master[13703:13907] + exit_join + master[13940:14517]
               + left_circle_1 + master[14555:15749] + left_circle_2
               + master[15806:15867] + left_circle_3 + master[15902:])
    ids = [p["id"] for p in revised]
    for removed in (range(12653, 12845), range(13643, 13704), range(13908, 13941),
                    range(14518, 14556), range(15750, 15807), range(15868, 15903)):
        if any(f"W{n}" in ids for n in removed):
            raise AssertionError("An omitted W remains in the revision")
    for marker in ("W12652", "M18", "M2860", "W12845", "W13642", "W13704", "W13907", "W13941",
                   "W14517", "W14556", "W15749", "W15807", "W15867", "W15903"):
        if ids.count(marker) != 1:
            raise AssertionError(f"Missing or duplicate {marker}")
    steps = [math.dist(a["xy"], b["xy"]) for a, b in zip(revised, revised[1:])]
    if max(steps) > 0.55:
        raise ValueError(f"Revision has an unexpectedly large waypoint gap: {max(steps):.3f} m")

    OUT.mkdir(parents=True, exist_ok=True)
    mission_file = OUT / "62_Collins_master_manual_revision_DRAFT_1mps_20260920.txt"
    audit_file = OUT / "62_Collins_master_manual_revision_audit_20260920.csv"
    report_file = OUT / "62_Collins_master_manual_revision_report_20260920.json"
    figure_file = OUT / "62_Collins_master_manual_revision_REVIEW_20260920.png"
    left_figure_file = OUT / "62_Collins_garden_left_cuts_REVIEW_20260920.png"
    with mission_file.open("w", encoding="utf-8", newline="\n") as handle:
        for p in revised:
            handle.write(f'{p["lat"]:.9f} {p["lon"]:.9f} {bearing_to_yaw(p["heading"]):.6f} {LOOKAHEAD_M:.2f} {SPEED_MPS:.2f}\n')
    with audit_file.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["revision_waypoint", "source_id", "phase", "lat", "lon", "east_m", "north_m", "heading_deg", "speed_mps", "gps_time", "gps_fix", "field_log_row"])
        writer.writeheader()
        for i, p in enumerate(revised, 1):
            writer.writerow(dict(revision_waypoint=i, source_id=p["id"], phase=p.get("phase", "recorded_manual"),
                                 lat=f'{p["lat"]:.9f}', lon=f'{p["lon"]:.9f}', east_m=f'{p["xy"][0]:.3f}',
                                 north_m=f'{p["xy"][1]:.3f}', heading_deg=f'{p["heading"]:.3f}', speed_mps=SPEED_MPS,
                                 gps_time=p.get("time", ""), gps_fix=p.get("fix_quality", ""),
                                 field_log_row=p.get("field_row", "")))
    report = dict(status="REVIEW_ONLY_NOT_APPROVED_FOR_TRACTOR", source_mission=str(MISSION.relative_to(SITE)),
                  source_mission_sha256=sha256(MISSION), source_field_log=str(FIELD_LOG.relative_to(SITE)),
                  source_field_log_sha256=sha256(FIELD_LOG), original_waypoints=len(master),
                  revised_waypoints=len(revised), manual_points=len(selected),
                  manual_fix_quality=dict(Counter(p["fix_quality"] for p in selected)),
                  removed_original_ranges=[[12653,12844],[13643,13703],[13908,13940],
                                           [14518,14555],[15750,15806],[15868,15902]],
                  connectors={"W12652_to_M18":len(entry),"M2860_to_W12845":len(ret),
                              "W13642_to_W13704":len(garden),"W13907_to_W13941":len(exit_join),
                              "W14517_to_W14556":len(left_circle_1),
                              "W15749_to_W15807":len(left_circle_2),
                              "W15867_to_W15903":len(left_circle_3)},
                  garden_left_cut_direct_distance_m={f"W{a}_to_W{b}":round(math.dist(w(a)["xy"],w(b)["xy"]),3)
                                                     for a,b in ((14517,14556),(15749,15807),(15867,15903))},
                  max_consecutive_gap_m=max(steps), speed_policy="All output rows set to 1.00 m/s for review, including tight curves; NOT field-approved.",
                  unresolved=["Return connector terrain/obstacle clearance not field-verified",
                              "New connector curvature and controller tracking not field-validated",
                              "W12652-to-M18 entry changes heading by about 28 degrees in only 0.82 m",
                              "W13704 resumes with a heading about 34 degrees left of the W13642-to-W13704 shortcut bearing",
                              "22 recorded manual points have DGPS, not RTK Fixed",
                              "Three garden-left circle-removal joins still require heading/clearance and controller-tracking review",
                              "Garden-right center stripes not included; turn geometry requires separate review"])
    report_file.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    def path(ax, arr, color, label, lw=1.5, style="-"):
        ax.plot([q["xy"][0] for q in arr], [q["xy"][1] for q in arr], style,
                color=color, lw=lw, label=label)
    left, right = axes
    path(left, master[12520:12900], "#9aa8b3", "Original W route", 1.1)
    path(left, selected, "#15803d", "M18–M2860 recorded manual", 2)
    path(left, [w(12652), *entry, selected[0]], "#e11d48", "Entry join", 2)
    path(left, [selected[-1], *ret, w(12845)], "#c026d3", "Draft return join", 2)
    for name,p in [("W12652",w(12652)),("M18",selected[0]),("M2860",selected[-1]),("W12845",w(12845))]:
        left.scatter(*p["xy"], s=30, color="black", zorder=5)
        left.annotate(name,p["xy"],xytext=(5,5),textcoords="offset points",fontsize=9)
    left.set_title("Backyard manual splice and 21-m return")
    for k in (1,2,3):
        path(right, [p for p in master if p["phase"] == f"garden_right_ring_{k}"],
             ["#94a3b8","#64748b","#2563eb"][k-1], f"Garden ring {k}", 1.5)
    path(right, master[13641:13704], "#f59e0b", "Original W13642–W13704", 1, "--")
    path(right, [w(13642),*garden,w(13704)], "#e11d48", "Draft shortcut 1", 2.5)
    path(right, master[13906:13941], "#f59e0b", "Original W13907–W13941", 1, "--")
    path(right, [w(13907),*exit_join,w(13941)], "#c026d3", "Draft shortcut 2", 2.5)
    for name,p in [("W13642",w(13642)),("W13704",w(13704)),("W13907",w(13907)),("W13941",w(13941))]:
        right.scatter(*p["xy"], s=25, color="black", zorder=5)
        right.annotate(name,p["xy"],xytext=(5,5),textcoords="offset points",fontsize=8)
    right.set_title("Garden-right shortcuts; center remains unplanned")
    for ax in axes:
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.25)
        ax.set_xlabel("East (m, local)")
        ax.set_ylabel("North (m, local)")
    left.legend(fontsize=8, loc="best")
    right.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.02, 1))
    fig.suptitle("REVIEW ONLY — not cleared for Auto / blades", fontsize=15, color="#b91c1c")
    fig.savefig(figure_file, dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(17, 5), constrained_layout=True)
    for ax, (a, b, connector) in zip(axes, ((14517,14556,left_circle_1),
                                            (15749,15807,left_circle_2),
                                            (15867,15903,left_circle_3))):
        path(ax, master[a-1:b], "#9aa8b3", "Original circle", 1.6)
        path(ax, [w(a), *connector, w(b)], "#c026d3", "New direct join", 2.5)
        for n in (a,b):
            ax.scatter(*w(n)["xy"], s=38, color="black", zorder=5)
            ax.annotate(f"W{n}",w(n)["xy"],xytext=(6,5),textcoords="offset points",fontsize=10)
        ax.set_title(f"W{a} → W{b}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.25)
        ax.set_xlabel("East (m, local)")
        ax.set_ylabel("North (m, local)")
        ax.legend(fontsize=8)
    fig.suptitle("Garden-left circle removals — REVIEW ONLY", color="#b91c1c")
    fig.savefig(left_figure_file, dpi=180)
    plt.close(fig)
    print(json.dumps({"out":str(OUT),"waypoints":len(revised),"manual_points":len(selected),
                      "connectors":report["connectors"],"max_gap_m":round(max(steps),3)},indent=2))


if __name__ == "__main__":
    main()
