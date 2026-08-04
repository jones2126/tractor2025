# Northern-slope straight steering test — 2026-08-04

This package contains four **separate**, straight-only missions. It does not
contain autonomous turns. Use it to observe steering/PID behavior on the
northern slope before attempting another coverage mission.

| Test | Direction | Length | Speed | Compass heading |
|---|---:|---:|---:|---:|
| `1E` | East | 20.5 m | 0.50 m/s | 090° |
| `1W` | West | 20.5 m | 0.50 m/s | 270° |
| `2E` | East | 25.5 m | 0.50 m/s | 090° |
| `2W` | West | 25.5 m | 0.50 m/s | 270° |

The lines are 6 m and 8 m south of the newly recorded northernmost point. Both
endpoints are generated from an area inset 3.0 m from the complete manually
driven lap. The source log lost RTK Fixed intermittently, including on the
final approach, so these are supervised test lines—not a finalized replacement
polygon.

## Field procedure

1. Keep the mower deck disengaged and the switch UP in Pause.
2. Run `/home/al/mission_preflight_20260804.py` and require a PASS.
3. Manually position the tractor near the selected mission's printed start
   coordinate and align it with the printed compass heading.
4. Start one test, for example:

   ```bash
   cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/steering_tests/20260804_north
   bash ./run_north_straight_test_20260804.sh 1E
   ```

5. Leave the switch in Pause while the logger/controller starts. Review the
   displayed mission and controller state before selecting Auto.
6. Select Pause at the end. Reposition manually before starting another test.

Recommended sequence: `1E`, `1W`, `2E`, `2W`. Stop after the first pair if the
steering response is unsafe or the IBT-2 latches.

Each test writes a uniquely named 20 Hz field log under `/home/al/field_logs`.
The Pure Pursuit controller independently writes its 20 Hz pursuit log under
`/home/al/repos/field-testing-data`.
