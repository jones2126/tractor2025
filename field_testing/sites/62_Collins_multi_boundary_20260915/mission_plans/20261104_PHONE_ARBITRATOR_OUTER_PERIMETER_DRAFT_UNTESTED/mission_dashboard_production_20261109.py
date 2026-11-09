#!/usr/bin/env python3
"""Enabled production dashboard for the approved blades-off perimeter run."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
SOURCE = REPO / "tractor_rpi/pure-pursuit/mission_dashboard_outer_perimeter_field_20260928.py"
spec = importlib.util.spec_from_file_location("outer_dashboard", SOURCE)
outer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(outer)
dashboard = outer.dashboard
dashboard.REPO = REPO
dashboard.LAUNCHER = HERE / "run_outer_perimeter_20261104.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN PHONE OUTER PERIMETER BLADES OFF"

legacy_safe_to_start = dashboard.safe_to_start


def production_safe_to_start(state):
    ok, reason = legacy_safe_to_start(state)
    if not ok:
        return ok, reason
    snap = state.snapshot()
    arbitrator = snap["bridge"].get("rpi_arbitrator", {})
    if arbitrator.get("requested_mode") != "PAUSE":
        return False, "Phone request is not Pause"
    if arbitrator.get("actual_serial_output") != "PAUSE":
        return False, "RPi safety controller actual output is not Pause"
    if arbitrator.get("phone_link_fresh") is not True:
        return False, "Direct phone control link is not fresh"
    if arbitrator.get("scheduler_fault_latched") is True:
        return False, "RPi-to-Teensy command scheduler has a latched fault"
    if arbitrator.get("additional_local_faults"):
        return False, "Local actuator health fault remains active"
    if arbitrator.get("communications_degraded_preauthorized") is not True:
        return False, "Bounded communications-degraded policy is not active"
    if arbitrator.get("distance_envelope_source") != "ROUTE_PROGRESS_ONLY":
        return False, "Degraded distance source is not route progress"
    return True, ""


dashboard.safe_to_start = production_safe_to_start


def replace_once(text: str, original: str, updated: str) -> str:
    if text.count(original) != 1:
        raise RuntimeError(f"Dashboard source changed; expected one occurrence: {original}")
    return text.replace(original, updated)


dashboard.HTML = replace_once(
    dashboard.HTML,
    "Tractor01 — outer-perimeter field test",
    "Tractor01 — production phone-controlled outer perimeter",
)
dashboard.HTML = replace_once(
    dashboard.HTML,
    "RUN OUTER PERIMETER BLADES OFF",
    dashboard.EXPECTED_CONFIRMATION,
)
dashboard.HTML = replace_once(
    dashboard.HTML,
    "const wifi=b.wifi_control||{},wifiPresent=Object.keys(wifi).length>0;",
    "const wifi=b.wifi_control||{},wifiPresent=Object.keys(wifi).length>0,arb=b.rpi_arbitrator||{},arbPresent=Object.keys(arb).length>0;",
)
dashboard.HTML = replace_once(
    dashboard.HTML,
    "const nowMs=Date.now(),instantLinkFresh=!wifiPresent||(Number(wifi.heartbeat_fresh)===1&&Number(wifi.estop_latched)===0&&Number(wifi.command_age_ms)<=500);",
    "const nowMs=Date.now(),legacyInstantLinkFresh=!wifiPresent||(Number(wifi.heartbeat_fresh)===1&&Number(wifi.estop_latched)===0&&Number(wifi.command_age_ms)<=500),instantLinkFresh=arbPresent?(arb.phone_link_fresh===true&&arb.scheduler_fault_latched!==true&&Array.isArray(arb.additional_local_faults)&&arb.additional_local_faults.length===0):legacyInstantLinkFresh;",
)
dashboard.HTML = replace_once(
    dashboard.HTML,
    "const controllerStableMs=Number(c.operator_link_stable_s||0)*1000;",
    "const controllerStableMs=arbPresent?browserStableMs:Number(c.operator_link_stable_s||0)*1000;",
)
dashboard.HTML = replace_once(
    dashboard.HTML,
    "const autoArmReady=active&&controllerFresh&&bridgeFresh&&gpsFresh&&!softwarePaused&&phonePaused&&controlLinkReady&&waitReason.includes('mode=PAUSE');",
    "const ah=arb.health||{},productionSafetyReady=!arbPresent||(arb.requested_mode==='PAUSE'&&arb.actual_serial_output==='PAUSE'&&arb.phone_link_fresh===true&&arb.scheduler_fault_latched!==true&&Array.isArray(arb.additional_local_faults)&&arb.additional_local_faults.length===0&&ah.teensy_status_fresh===true&&ah.teensy_command_watchdog_healthy===true&&ah.pure_pursuit_command_fresh===true&&ah.controller_telemetry_fresh===true&&ah.gps_fresh===true&&ah.rtk_fixed===true&&ah.heading_valid_fixed===true&&ah.route_defined===true&&ah.inside_route_corridor===true&&ah.speed_within_limit===true);const autoArmReady=active&&controllerFresh&&bridgeFresh&&gpsFresh&&!softwarePaused&&phonePaused&&controlLinkReady&&productionSafetyReady&&(arbPresent||waitReason.includes('mode=PAUSE'));",
)

dashboard.MESSAGES_HTML = replace_once(
    dashboard.MESSAGES_HTML,
    "reason=c.wait_reason||d.last_safety_stop_reason||'None'",
    "reason=c.wait_reason||d.current_exact_reason||d.last_safety_stop_reason||'None'",
)


if __name__ == "__main__":
    print("PRODUCTION FIELD PACKAGE: exact reviewed route; blades must remain off.")
    dashboard.main()
