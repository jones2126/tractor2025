#!/usr/bin/env python3
"""Read-only, standard-library verification of the 62 Collins field package."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
GENERATED = HERE / "generated_master_manual_field_20260920"
STEM = "62_Collins_master_manual_resampled_1mps_20260920"
MISSION = GENERATED / f"{STEM}.txt"
AUDIT = GENERATED / f"{STEM}_audit.csv"
REPORT = GENERATED / f"{STEM}_report.json"
EXPECTED_SHA256 = "0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4"
EXPECTED_ROWS = 19_825
EXPECTED_CUTS = [[12653,12844],[13643,13703],[13908,13940],
                 [14518,14555],[15750,15806],[15868,15902]]
PAIRS = (("W12652","M18"),("M2860","W12845"),("W13642","W13704"),
         ("W13907","W13941"),("W14517","W14556"),
         ("W15749","W15807"),("W15867","W15903"))


def normalized_bytes(path: Path) -> bytes:
    return path.read_bytes().replace(b"\r\n",b"\n")


def verify() -> None:
    for path in (MISSION,AUDIT,REPORT):
        if not path.is_file():
            raise ValueError(f"Required field package file missing: {path}")
    mission_bytes = normalized_bytes(MISSION)
    digest = hashlib.sha256(mission_bytes).hexdigest()
    if digest != EXPECTED_SHA256:
        raise ValueError(f"Mission checksum {digest} differs from approved package checksum")
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    if report.get("status") != "SUPERVISED_BLADES_OFF_FIELD_TEST":
        raise ValueError("Package status is not supervised blades-off field test")
    if report.get("mission_sha256") != digest or report.get("waypoints") != EXPECTED_ROWS:
        raise ValueError("Report disagrees with mission")
    if report.get("preserved_review_cuts") != EXPECTED_CUTS:
        raise ValueError("Reviewed cut ranges changed")
    if report.get("manual_resampled_points") != 876 or report.get("manual_hausdorff_deviation_m",1) > 0.05:
        raise ValueError("Manual-trace resampling is outside reviewed limits")
    audit_bytes = normalized_bytes(AUDIT)
    if report.get("audit_sha256") != hashlib.sha256(audit_bytes).hexdigest():
        raise ValueError("Audit checksum differs from report")
    rows = [line.split() for line in mission_bytes.decode("utf-8").splitlines()]
    if len(rows) != EXPECTED_ROWS or any(len(row) != 5 for row in rows):
        raise ValueError("Mission row count or column count changed")
    if any(not all(math.isfinite(float(v)) for v in row) for row in rows):
        raise ValueError("Mission has a non-finite value")
    if any(row[3:] != ["2.00","1.00"] for row in rows):
        raise ValueError("Mission lookahead/speed policy changed")
    with AUDIT.open(newline="",encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    if len(audit) != EXPECTED_ROWS:
        raise ValueError("Audit row count changed")
    ids = [row.get("source_id","") for row in audit]
    if len(set(ids)) != EXPECTED_ROWS or ids[0] != "W1" or ids[-1] != "W19250":
        raise ValueError("Source lineage or endpoints changed")
    if any(row.get("waypoint") != str(i) or not row.get("phase")
           for i,row in enumerate(audit,1)):
        raise ValueError("Audit indices or phases invalid")
    if any(abs(float(audit[i][key])-float(rows[i][j])) > 1e-9
           for i in range(EXPECTED_ROWS) for key,j in (("lat",0),("lon",1),("yaw_rad",2))):
        raise ValueError("Audit and mission coordinates/yaws differ")
    if any(row.get("speed_mps") != "1.00" or row.get("lookahead_m") != "2.00" for row in audit):
        raise ValueError("Audit speed or lookahead changed")
    for first,last in PAIRS:
        a,b=ids.index(first),ids.index(last)
        if a >= b or any(item.startswith("W") for item in ids[a+1:b]):
            raise ValueError(f"Reviewed join {first} to {last} is not intact")
    if ids.count("M18") != 1 or ids.count("M2860") != 1:
        raise ValueError("Manual endpoints changed")
    if sum(item.startswith("S") and item[1:].isdigit() for item in ids) != 874:
        raise ValueError("Resampled manual point count changed")
    lat0 = float(rows[0][0])
    x_scale = 111_320*math.cos(math.radians(lat0))
    max_gap = 0.0
    for a,b in zip(rows,rows[1:]):
        gap = math.hypot((float(b[1])-float(a[1]))*x_scale,
                         (float(b[0])-float(a[0]))*111_132)
        max_gap = max(max_gap,gap)
    if max_gap > 0.55:
        raise ValueError(f"Mission waypoint gap {max_gap:.3f} m exceeds limit")
    print("PASS: exact reviewed and resampled field mission verified.")
    print(f"      {EXPECTED_ROWS:,} waypoints, 2.00 m lookahead, every speed command 1.00 m/s.")
    print("      All six reviewed W ranges omitted; M18/M2860 preserved with 874 resampled interior points.")
    print(f"      Manual trace maximum geometric deviation {report['manual_hausdorff_deviation_m']:.3f} m; maximum waypoint gap {max_gap:.3f} m.")


if __name__ == "__main__":
    try:
        verify()
    except (ValueError,KeyError,IndexError,OSError) as exc:
        raise SystemExit(f"FIELD PACKAGE VERIFICATION FAIL: {exc}")
