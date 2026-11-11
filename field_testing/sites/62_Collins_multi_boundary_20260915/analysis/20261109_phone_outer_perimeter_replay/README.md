# November 9 mission replay

Open `replay_20261109_phone_outer_perimeter.html` in a modern browser. It is self-contained and makes no network requests.

`safety_pause_timeline.csv` groups each complete interruption from the first non-AUTO/AUTO replay sample through the first restored AUTO/AUTO sample. Replay sample numbers are not route waypoint numbers; both are included explicitly. Route waypoint indices are zero-based, matching the replay display and Pure Pursuit log. `pause_transition_details.csv` retains the individual low-level Pause transitions. `failure_duration_s` measures the continuous breached condition from its first bad raw 20 Hz Pure Pursuit row through the first recovered row; it is blank for an operator-selected Pause. This is distinct from the longer interruption/re-arming `duration_s`.

Rebuild from the repository root with:

```bash
python3 field_testing/tools/build_tractor01_mission_replay_20261109.py --evidence-root /path/to/20261109_phone_outer_perimeter_collection
```
