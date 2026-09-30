#!/usr/bin/env python3
"""Verify the exact approved 2026-09-29 consolidated perimeter package."""
import csv, hashlib, json, math
from pathlib import Path
HERE=Path(__file__).resolve().parent
G=HERE/"generated"
STEM="62_Collins_consolidated_perimeter_1mps_20260929"
MISSION=G/f"{STEM}.txt"; AUDIT=G/f"{STEM}_audit.csv"; REPORT=G/f"{STEM}_validation.json"
DASHBOARD=HERE/"mission_dashboard_consolidated_perimeter_20260929.py"
LAUNCHER=HERE/"run_62_Collins_consolidated_perimeter_20260929.sh"
EXPECTED_MISSION_SHA256="35ea19776283ef415518fd57997e418b966ea823741bcc6a6ca135cee7b504d9"
EXPECTED_AUDIT_SHA256="fe2d65daaecd80b4dce1582a4e74ab4744c0fe7cb1cf9e72d6944f846353e50e"
EXPECTED_ROWS=3989
EXPECTED_LENGTH_M=790.077117
EXPECTED_STATUS="SUPERVISED_BLADES_OFF_FIELD_TEST_APPROVED_20260929"
def digest(path): return hashlib.sha256(path.read_bytes().replace(b"\r\n",b"\n")).hexdigest()
def main():
    for path in (MISSION,AUDIT,REPORT,DASHBOARD,LAUNCHER):
        if not path.is_file(): raise ValueError(f"missing required file: {path}")
    if digest(MISSION)!=EXPECTED_MISSION_SHA256: raise ValueError("mission checksum changed")
    if digest(AUDIT)!=EXPECTED_AUDIT_SHA256: raise ValueError("audit checksum changed")
    report=json.loads(REPORT.read_text(encoding="utf-8"))
    if report.get("status")!=EXPECTED_STATUS: raise ValueError("package status changed")
    if report.get("mission_sha256")!=EXPECTED_MISSION_SHA256 or report.get("audit_sha256")!=EXPECTED_AUDIT_SHA256: raise ValueError("report checksums disagree")
    rows=[line.split() for line in MISSION.read_text(encoding="ascii").splitlines()]
    if len(rows)!=EXPECTED_ROWS or any(len(row)!=5 for row in rows): raise ValueError("mission row or column count changed")
    if any(not all(math.isfinite(float(v)) for v in row) for row in rows): raise ValueError("non-finite mission value")
    if any(row[4]!="1.00" for row in rows): raise ValueError("speed must remain 1.00 m/s")
    with AUDIT.open(newline="",encoding="utf-8-sig") as handle: audit=list(csv.DictReader(handle))
    if len(audit)!=EXPECTED_ROWS: raise ValueError("audit row count changed")
    if report["geometry_validation"]["duplicate_consecutive_points"]!=0: raise ValueError("duplicate points reported")
    if report["geometry_validation"]["maximum_waypoint_gap_m"]>0.202: raise ValueError("waypoint gap exceeds 0.202 m")
    if report["geometry_validation"]["reversal_events_over_150_deg"]: raise ValueError("instantaneous reversal reported")
    if report["geometry_validation"]["heading_change_events_over_45_deg"]: raise ValueError("implausible per-waypoint heading change reported")
    if report["deck_envelope_validation"]["intersecting_obstacles"]: raise ValueError("deck envelope intersects an obstacle exclusion")
    if abs(report["candidate_route"]["length_m"]-EXPECTED_LENGTH_M)>0.001: raise ValueError("route length changed")
    dashboard_text=DASHBOARD.read_text(encoding="utf-8")
    if not all(marker in dashboard_text for marker in ("/api/note","tractor\\s+note","NOTE_JSONL","record_voice_note")): raise ValueError("hands-free voice-note dashboard feature is missing")
    if "APPROVED_FOR_FIELD=true" not in LAUNCHER.read_text(encoding="utf-8"): raise ValueError("approved launcher gate is not enabled")
    print("PASS: exact approved supervised blades-off field package verified.")
    print(f"      {EXPECTED_ROWS:,} waypoints; {EXPECTED_LENGTH_M:.3f} m; all speed commands 1.00 m/s.")
    print("      Approved for the initial directly supervised blades-off field test only.")
if __name__=="__main__":
    try: main()
    except (ValueError,KeyError,OSError) as exc: raise SystemExit(f"REVIEW PACKAGE VERIFICATION FAIL: {exc}")
