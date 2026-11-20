#!/usr/bin/env python3
# Relocated to tools/ on 2026-11-20; export behavior unchanged.
"""Export a time-bounded field CSV from verified closed telemetry segments."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path


def timestamp(value):
    parsed = dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("Use an ISO timestamp with timezone")
    return parsed


def verify(directory):
    manifest = directory / "SHA256SUMS"
    names = set()
    for line in manifest.read_text().splitlines():
        digest, name = line.split("  ", 1)
        if Path(name).name != name or name in names:
            raise ValueError("Invalid checksum manifest")
        names.add(name)
        hasher = hashlib.sha256()
        with (directory / name).open("rb") as handle:
            for block in iter(lambda: handle.read(1048576), b""):
                hasher.update(block)
        if hasher.hexdigest() != digest:
            raise ValueError("Checksum mismatch: " + str(directory / name))
    if not {"metadata.json", "field.csv", "imu.jsonl", "encoders.jsonl", "gnss.jsonl", "status.jsonl"} <= names:
        raise ValueError("Incomplete segment manifest")
    metadata = json.loads((directory / "metadata.json").read_text())
    if metadata["state"] not in ("ROTATED", "STOPPED"):
        raise ValueError("Segment was not closed successfully")
    return hashlib.sha256(manifest.read_bytes()).hexdigest()


def export(segments, output, start, end):
    if start >= end:
        raise ValueError("Start must precede end")
    # Verify every source before creating any output. Never modify raw files.
    sources = {str(p): verify(p) for p in segments}
    header = None
    rows = []
    for segment in segments:
        with (segment / "field.csv").open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if header is not None and reader.fieldnames != header:
                raise ValueError("Mixed field schemas")
            header = reader.fieldnames
            for row in reader:
                if start <= timestamp(row["time"]) <= end:
                    rows.append(row)
    if not rows:
        raise ValueError("No field rows in requested window")
    rows.sort(key=lambda row: timestamp(row["time"]))
    output.mkdir(parents=True, exist_ok=False)
    target = output / "field.csv"
    with target.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, header)
        writer.writeheader()
        writer.writerows(rows)
    report = dict(schema=1, start=start.isoformat(), end=end.isoformat(), field_rows=len(rows),
                  sources=sources, source_manifest_hash_algorithm="SHA256",
                  note="Raw IMU/encoder/status evidence remains in the listed source segments; export is a derived CSV, not a new raw recording")
    (output / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    sums = [hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name for p in sorted(output.iterdir())]
    (output / "SHA256SUMS").write_text("\n".join(sums) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--segment", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start", required=True)
    parser.add_argument("--end", required=True)
    args = parser.parse_args(argv)
    print(json.dumps(export(args.segment, args.output, timestamp(args.start), timestamp(args.end)), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
