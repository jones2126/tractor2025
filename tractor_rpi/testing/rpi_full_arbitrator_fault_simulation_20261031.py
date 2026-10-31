#!/usr/bin/env python3
"""Deterministic, hardware-free fault matrix for the full RPi arbitrator."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from tractor_rpi.rpi_arbitrator_inputs_20261031 import (
    PhoneIntentSample,
    PurePursuitCommandSample,
    PurePursuitHealthSample,
    RpiArbitratorInputCollector,
    TeensySafetySample,
)
from tractor_rpi.rpi_command_lease_adapter_20261031 import parse_rpi_ctl_line
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode
from tractor_rpi.rpi_safety_supervisor_20261031 import RpiSafetySupervisor, SupervisorState


ROUTE = "a" * 64


def status(*, usable: int = 1, latched: int = 0):
    return parse_rpi_ctl_line(
        f"1,100,RPI_CTL,m=2,f=1,u={usable},l={latched},a=10,"
        "ss=42,s=10,ok=10,rj=0,rr=0"
    )


def refresh_local(
    collector: RpiArbitratorInputCollector,
    now: float,
    sequence: int,
    *,
    progress: float,
    route: str = ROUTE,
    carrier: str = "fixed",
    steering_healthy: bool = True,
    lease_usable: int = 1,
    expiry_latched: int = 0,
) -> None:
    collector.ingest_pure_pursuit_command(
        PurePursuitCommandSample(now, sequence, route, 1.0, 0.2)
    )
    collector.ingest_pure_pursuit_health(
        PurePursuitHealthSample(
            now, route, "RUNNING", 0.05, "RTK Fixed", True, carrier, 0.2,
            progress, 0.25,
        )
    )
    collector.ingest_teensy(
        TeensySafetySample(
            now,
            status(usable=lease_usable, latched=expiry_latched),
            True,
            True,
            steering_healthy,
            True,
        )
    )


def new_system():
    collector = RpiArbitratorInputCollector(
        expected_route_sha256=ROUTE,
        degraded_preauthorized=True,
    )
    collector.ingest_phone(
        PhoneIntentSample(0.0, RpiCommandMode.AUTO_MPS, "phone", 1)
    )
    refresh_local(collector, 0.0, 1, progress=0.0)
    return collector, RpiSafetySupervisor()


def record(name, result, expected):
    passed = result.state == expected
    return {
        "case": name,
        "pass": passed,
        "expected": expected.value,
        "actual": result.state.value,
        "reason": result.reason,
        "incident_id": None if result.guard is None else result.guard.incident_id,
        "elapsed_s": None if result.guard is None else result.guard.elapsed_s,
        "distance_m": None if result.guard is None else result.guard.distance_m,
    }


def run_matrix():
    rows = []
    c, s = new_system()
    rows.append(record("nominal_auto", s.evaluate(c.build(now_monotonic_s=0.1)), SupervisorState.AUTO))

    refresh_local(c, 0.6, 2, progress=0.6)
    degraded = s.evaluate(c.build(now_monotonic_s=0.6))
    rows.append(record("phone_loss_only", degraded, SupervisorState.COMMUNICATIONS_DEGRADED))
    incident = degraded.guard.incident_id

    c.ingest_phone(PhoneIntentSample(5.0, RpiCommandMode.AUTO_MPS, "phone", 2))
    refresh_local(c, 5.0, 3, progress=5.0)
    recovered = s.evaluate(c.build(now_monotonic_s=5.0))
    row = record("phone_reconnect_budget_sticky", recovered, SupervisorState.COMMUNICATIONS_DEGRADED)
    row["pass"] = row["pass"] and recovered.guard.incident_id == incident
    rows.append(row)

    refresh_local(c, 30.6, 4, progress=29.0)
    rows.append(record("time_envelope_exhausted", s.evaluate(c.build(now_monotonic_s=30.6)), SupervisorState.PAUSE_LATCHED))

    c, s = new_system()
    refresh_local(c, 0.6, 2, progress=0.6)
    s.evaluate(c.build(now_monotonic_s=0.6))
    refresh_local(c, 20.0, 3, progress=30.6)
    rows.append(record("distance_envelope_exhausted", s.evaluate(c.build(now_monotonic_s=20.0)), SupervisorState.PAUSE_LATCHED))

    fault_cases = (
        ("route_hash_mismatch", {"route": "b" * 64}),
        ("heading_not_fixed", {"carrier": "float"}),
        ("steering_fault", {"steering_healthy": False}),
        ("teensy_lease_expired", {"lease_usable": 0, "expiry_latched": 1}),
    )
    for name, kwargs in fault_cases:
        c, s = new_system()
        refresh_local(c, 0.6, 2, progress=0.6, **kwargs)
        rows.append(record(name, s.evaluate(c.build(now_monotonic_s=0.6)), SupervisorState.PAUSE_LATCHED))

    c, s = new_system()
    c.ingest_phone(PhoneIntentSample(0.1, RpiCommandMode.MANUAL_PERCENT, "phone", 2))
    # Let only the phone expire; keep Teensy safety fresh.
    c.ingest_teensy(TeensySafetySample(0.7, status(), True, True, True, True))
    rows.append(record("manual_phone_loss", s.evaluate(c.build(now_monotonic_s=0.7)), SupervisorState.PAUSE_LATCHED))
    return rows


def main() -> int:
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    rows = run_matrix()
    text = "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n"
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")
    sys.stdout.write(text)
    failed = [row for row in rows if not row["pass"]]
    print(f"{'PASS' if not failed else 'FAIL'}: {len(rows) - len(failed)}/{len(rows)} arbitrator fault cases")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
