#!/usr/bin/env python3
"""Verify a closed candidate capture without hardware access or source changes."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path

REQUIRED={"samples.csv","encoders.csv","gnss.jsonl","shadow_pose.jsonl","events.jsonl","metadata.json"}


def verify(root):
    manifest=root/"SHA256SUMS"
    names=set()
    for line in manifest.read_text(encoding="utf-8").splitlines():
        expected,name=line.split("  ",1)
        if name not in REQUIRED or name in names:raise ValueError("Unexpected manifest path or duplicate entry")
        digest=hashlib.sha256()
        with (root/name).open("rb") as handle:
            for block in iter(lambda:handle.read(1048576),b""):digest.update(block)
        if digest.hexdigest()!=expected:raise ValueError(f"Checksum mismatch: {name}")
        names.add(name)
    if names!=REQUIRED:raise ValueError("Missing required capture file")
    metadata=json.loads((root/"metadata.json").read_text())
    if metadata["status"]!="STOPPED":raise ValueError(f"Capture did not stop cleanly: {metadata['status']}")
    if metadata.get("imu_calibration") is None:raise ValueError("No passed IMU calibration")
    counts={};last=None;phases=set()
    with (root/"samples.csv").open(newline="",encoding="utf-8") as handle:
        rows=0
        for row in csv.DictReader(handle):
            value=float(row["monotonic_s"])
            if last is not None and value<=last:raise ValueError("Non-increasing IMU times")
            last=value;rows+=1;phases.add(row["phase"])
        counts["imu"]=rows
    if "observation" not in phases:raise ValueError("No post-calibration IMU samples")
    sides={"L":0,"R":0}
    with (root/"encoders.csv").open(newline="",encoding="utf-8") as handle:
        for row in csv.DictReader(handle):sides[row["side"]]+=1
    if not all(v>=2 for v in sides.values()):raise ValueError("One or both wheel streams missing")
    counts["encoders"]=sum(sides.values())
    for channel,name in (("gnss","gnss.jsonl"),("pose","shadow_pose.jsonl"),("events","events.jsonl")):
        count=0
        with (root/name).open(encoding="utf-8") as handle:
            for line in handle:
                record=json.loads(line);count+=1
                if channel=="pose" and record["pose"]["valid_for_control"] is not False:raise ValueError("Unexpected authoritative pose")
                if channel=="gnss" and any(k in record["message"] for k in ("pose","imu","encoders","wifi_ssid","wifi_rssi_dbm","wifi_signal_label")):
                    raise ValueError("Legacy GNSS schema mixed with experimental sensor fields")
        counts[channel]=count
    if counts!=metadata["record_counts"]:raise ValueError("Capture row counts disagree with metadata")
    if counts["imu"]!=metadata["imu_rows"]:raise ValueError("Queued IMU records were lost")
    if not all(metadata["udp_datagrams_sent"].get(str(port))==counts["gnss"] for port in (6002,6009,6010,6013)):
        raise ValueError("UDP publication counters disagree")
    return dict(result="CAPTURE FILE VERIFICATION PASS",counts=counts,wheel_samples=sides,
                commit=metadata["commit"],encoder_scales_calibrated=metadata["odometry_scale_verified"],
                limits="File integrity and presence only; not physical calibration or proof of UDP reception")


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("capture_dir",type=Path)
    args=parser.parse_args()
    try:print(json.dumps(verify(args.capture_dir),indent=2));return 0
    except (OSError,ValueError,KeyError) as exc:print(f"CAPTURE FILE VERIFICATION FAIL: {exc}");return 1


if __name__=="__main__":raise SystemExit(main())
