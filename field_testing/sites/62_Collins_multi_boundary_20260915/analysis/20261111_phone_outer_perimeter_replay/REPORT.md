# November 11, 2026 outer-perimeter mission analysis

**DRAFT - UNTESTED IN THE FIELD**

This is a historical reconstruction of recorded evidence. It does not authorize field motion or change any controller behavior.

## Recorded mission extent

- Pure Pursuit rows: **21,726**.
- Mission telemetry rows: **21,788**.
- Maximum route progress: **380.857 m**.
- Maximum waypoint index: **1917**.
- Maximum recorded cross-track error: **17.808 m**.

## Grouped interruptions

- Complete or recording-ended interruptions: **18**.
- Whole interruption/re-arm time: total **492.355 s**, median **18.602 s**, maximum **169.994 s**.
- Underlying breached-condition episodes: **22** (independent conditions can overlap).
- `pure_pursuit_safety`: **9** interruption(s).
- `supervisor_corridor`: **9** interruption(s).

The detailed start time, trigger, waypoint, total interruption duration, Pause latches, and re-arm attempts are in `safety_pause_timeline.csv`. Each distinct raw RTK, heading, GPS-feed, and corridor threshold breach and its duration is in `safety_condition_loss_episodes.csv`. Individual low-level Pause transitions remain in `pause_transition_details.csv`.

## Supporting observations

- Raw GPS/heading safety episodes detected: **10**.
- Individual output transitions into Pause: **21**.
- Phone freshness expiration events: **0**.
- Accepted-command gaps: median **200.0 ms**, p95 **218.7 ms**, maximum **463.2 ms**.
- Client-reported RTT: median **33.0 ms**, p95 **92.0 ms**, maximum **339.0 ms**.
- `heading_accuracy` breaches: **3**; total **1.001 s**, median **0.200 s**, maximum **0.601 s**.
- `heading_carrier` breaches: **6**; total **38.958 s**, median **2.980 s**, maximum **17.025 s**.
- `heading_valid` breaches: **2**; total **3.155 s**, median **1.577 s**, maximum **2.203 s**.
- `rtk_fix` breaches: **8**; total **23.634 s**, median **1.002 s**, maximum **9.113 s**.
- `supervisor_corridor` breaches: **3**; total **290.615 s**, median **107.402 s**, maximum **130.788 s**.
- JRK supply voltage minimum: **12.569 V** at `2026-11-11T19:24:33.789+00:00`.
- JRK supply voltage maximum: **14.535 V** at `2026-11-11T19:30:50.809+00:00`.
- JRK supply voltage final valid sample: **13.130 V** at `2026-11-11T19:31:18.621+00:00`.

## Interpretation boundary

The interruption categories are reconstructed by synchronizing the mission, Pure Pursuit, and phone-control logs. A category is evidence of the recorded gate that was breached; it is not by itself proof of the physical root cause. Any proposed safety-policy change requires separate design review and staged testing.

## Source integrity

- `outer_perimeter_20261111_141308.csv` — SHA-256 `BEB6D56567B3B582551B16C68A81AF7EE7303214712B57BA21E44C3F6EADCB61`
- `pursuit_log_20261111_141310.csv` — SHA-256 `8DFAE31688787DC51D9C4F60F82A5BE8CBCE2817AB9D4D38E20AF884EBF8F585`
- `wifi_primary_control_20261111_124338.jsonl` — SHA-256 `05F4D5D306013AF223FE704162B826DD9FFCBF5BDED0A8CA0E24D9873793ACE6`
- `62_Collins_outer_perimeter_field_test_1mps_20260928.txt` — SHA-256 `B47660B4C74D5E26CA5D9448F069FD7193C068CF62E3C0B62D5C083D9AED4D64`
- `62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv` — SHA-256 `4B7D093C42FC3CB47BF1F5A849458ADBA9FBF0D924FCDDE3CC15D3A7C6DABF1E`
