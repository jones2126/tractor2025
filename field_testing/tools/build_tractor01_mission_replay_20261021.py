#!/usr/bin/env python3
"""Build the October 21 Tractor01 mission replay from its evidence archive."""

from __future__ import annotations

import tarfile
import tempfile
from pathlib import Path

import build_tractor01_mission_replay_20260923 as builder


SITE = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
RUN = SITE / "runs" / "20261021_133807_master_manual_phone"
OUTPUT = SITE / "analysis" / "20261021_master_manual_phone_replay"
ARCHIVE = RUN / "tractor01_master_manual_endurance_20261021_151604.tgz"
OUTPUT_HTML = OUTPUT / "replay_20261021_133807_master_manual_phone.html"

PURSUIT_MEMBER = "./pursuit_logs/pursuit_log_20261021_133810.csv"
FIELD_MEMBER = "./mission_logs/master_manual_phone_20261021_133807.csv"


def main():
    if not ARCHIVE.is_file():
        raise FileNotFoundError(ARCHIVE)

    with tempfile.TemporaryDirectory(prefix="tractor01-replay-20261021-") as temporary:
        extracted = Path(temporary)
        with tarfile.open(ARCHIVE, "r:gz") as evidence:
            members = {member.name: member for member in evidence.getmembers()}
            required = (PURSUIT_MEMBER, FIELD_MEMBER)
            missing = [name for name in required if name not in members]
            if missing:
                raise ValueError(f"evidence archive is missing: {missing}")
            for name in required:
                evidence.extract(members[name], path=extracted, filter="data")

        builder.RUN = RUN
        builder.PURSUIT = extracted / PURSUIT_MEMBER
        builder.FIELD = extracted / FIELD_MEMBER
        builder.OUTPUT = OUTPUT
        builder.OUTPUT_HTML = OUTPUT_HTML
        builder.REQUIRED_EVENT_CATEGORIES = {
            "rtk", "heading", "mode", "driving", "reacquisition", "phase"
        }

        original_build_payload = builder.build_payload

        def build_payload():
            payload = original_build_payload()
            payload["title"] = (
                "October 21, 2026 · Interrupted and resumed Tractor01 mission · "
                "HISTORICAL REPLAY"
            )
            payload["run"] = RUN.name
            return payload

        builder.build_payload = build_payload
        builder.main()


if __name__ == "__main__":
    main()
