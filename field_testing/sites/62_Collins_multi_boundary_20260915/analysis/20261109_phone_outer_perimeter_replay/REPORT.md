# November 9, 2026 outer-perimeter mission analysis

## Confirmed result

The repeated stops were not caused by the direct phone network. The authoritative server log contains zero `phone_freshness_expired` events during this run. The interruptions came from GPS/heading safety gates and, later, the RPi supervisor's 2.0 m route-corridor gate.

## Counts

- Actual output transitions into Pause: **27**.
- Pure Pursuit GPS/heading safety stops while Auto was requested: **12**.
- Supervisor cross-track corridor latches: **11**.
- Operator-selected Pause transitions: **4**.
- Unresolved Pause transitions: **0**.
- Pure Pursuit safety episodes while Auto was requested: **12**.
- Phone mode transitions: **87** (`Auto→Pause` 27, `Pause→Manual` 20, `Manual→Auto` 16).
- Steering fault latch, steering drive block, and JRK halting errors: **zero in all field rows**.

## Phone and session evidence

- Accepted commands measured by the server: **9,278**.
- Accepted-command gap: median **200.1 ms**, p95 **214.6 ms**, p99 **239.1 ms**.
- Client-reported RTT: median **17.3 ms**, p95 **64.3 ms**, maximum **302.5 ms**.
- Phone freshness expirations: **0**.
- One 17.9 s accepted-command gap coincided with a new owner/session claim while the tractor was already in Pause. It did not initiate an Auto stop.
- The single `control session is not current` rejection occurred at server startup before the current owner was claimed.

## Safety parameters breached

Pure Pursuit required RTK Fixed, fixed/valid dual-antenna heading, heading accuracy no worse than 1.0°, and then five continuous healthy seconds before recovery. The recorded episodes included:

- RTK quality dropping to RTK Float or DGPS.
- Heading carrier dropping to float or none.
- `headValid`/RELPOSNED validity loss, including momentary zero-baseline records.
- Heading accuracy exceeding 1.0°.

Every such fault correctly stopped Pure Pursuit immediately and required a five-second stable recovery window. The RPi supervisor then remained Pause-latched until a fresh stopped Pause acknowledgement.

## Why Pause–Manual–Auto was sometimes needed

After progress 380.857 m, Pure Pursuit was willing to keep proposing within its 6.0 m tracking window, but the RPi supervisor required cross-track error ≤2.0 m. Recorded Auto attempts occurred at 2.003–5.403 m cross-track. Each Auto selection therefore immediately re-latched Pause. Manual was only useful for repositioning; it could not clear the latch unless the tractor returned to Pause while stopped, and Auto could not succeed until it was back inside 2.0 m.

This is a design mismatch, not a phone-link failure. The dashboard should display the exact latch reason and the distance/direction back to the permitted corridor instead of prompting repeated mode cycling.

## Mission extent

- Maximum route progress: **381.058 m** of 689.949 m.
- Maximum waypoint index: **1918** of 3461.
- Maximum cross-track error while Auto was requested: **5.617 m**.
- Maximum cross-track error across the full recording, including the later Manual drive home: **28.146 m**.

## Recommendations

1. Keep the supervisor's 2.0 m corridor limit for now; do not simply widen it to match the 6.0 m Pure Pursuit search window.
2. Make the controller and dashboard share one explicit recovery contract: exact breached field, current value, threshold, five-second stability countdown, Pause-latch state, and required recovery direction/distance.
3. When outside 2.0 m, inhibit Auto re-arm and provide Manual guidance back inside the corridor; only then accept stopped Pause and Auto.
4. Add continuous supervisor-decision logging. This run could be reconstructed, but exact latch reasons should be written as first-class events rather than inferred from synchronized feeds.
5. Preserve immediate GPS/heading and local actuator stops. Investigate antenna visibility and correction continuity before considering any threshold/hysteresis adjustment.
6. Re-run blades-off from the known route start after the recovery UI and decision logging are corrected.

## Source integrity

- `outer_perimeter_20261109_154324.csv` — SHA-256 `80C4BCACF61ED910F5C9E251AD798BEB7DE142958FA01EFAEFEDCD56D34B8F23`
- `pursuit_log_20261109_154326.csv` — SHA-256 `41C9796B3E8FB420A3FBDE9137EEFC24FFB9BFADCF9C4CD3569C004E0B2AD213`
- `wifi_primary_control_20261109_154125.jsonl` — SHA-256 `23C5C30EB1AAB56A88BF58E2CB9C579D4721126D37263406A618610705D0F84B`
- `62_Collins_outer_perimeter_field_test_1mps_20260928.txt` — SHA-256 `B47660B4C74D5E26CA5D9448F069FD7193C068CF62E3C0B62D5C083D9AED4D64`
- `62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv` — SHA-256 `54072D526370447F4EFD22A84052798543135FC45298E6BDA80E983178F5CF25`
