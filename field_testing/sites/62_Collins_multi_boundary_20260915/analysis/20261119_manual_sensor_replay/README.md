# November 19 manual replay

Open `replay_20261119_manual.html`. Use Play, the timeline, event menu and chart
window controls. All data is embedded; no server or internet connection is needed.
Wheel scale is provisional. See REPORT.md for definitions and limitations.

Rebuild from the independently verified preserved raw copy:

```powershell
python field_testing/tools/build_tractor01_sensor_replay_20261119.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261119_manual_sensor_replay
node field_testing/tools/check_sensor_replay_ui_20261119.cjs field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261119_manual_sensor_replay/replay_20261119_manual.html
```
