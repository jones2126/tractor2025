# DRAFT - UNTESTED IN THE FIELD

# Instrumentation deployment package

This package is prepared for the next stationary field session. It does not
enable `COMMUNICATIONS DEGRADED` and does not authorize movement.

Changed runtime files:

- `tractor_rpi/testing/webrtc/wifi_primary_control_20261022.py`
- `tractor_rpi/testing/webrtc/wifi_primary_control_20261022.html`
- `tractor_rpi/pure-pursuit/mission_dashboard_20260910.py`
- `tractor_rpi/communications_degraded_guard_20261029.py` (motion-free candidate)

After a commit is reviewed and pushed, the separately approved stationary
deployment sequence on Tractor01 is:

```bash
cd /home/al/tractor2025
git pull --ff-only origin main
python3 -m unittest \
  tractor_rpi.testing.test_wifi_primary_control_20261022 \
  tractor_rpi.testing.test_communications_degraded_guard_20261029 \
  tractor_rpi.testing.test_mission_dashboard_stale_state_20261019
sudo systemctl stop tractor-wifi-control.service
sudo bash tractor_rpi/setup/install_tractor_wifi_control_service.sh
sudo systemctl start tractor-wifi-control.service
systemctl is-active tractor-wifi-control.service
```

Before the stationary test, verify that
`/run/tractor-phone-control/diagnostics.json` is updating and that the
dashboard Messages view shows `DISABLED_PENDING_STATIONARY_APPROVAL` for
`COMMUNICATIONS DEGRADED`.

Do not start the mission launcher during Approval A. Keep physical Pause active,
engine off, blades off, and wheels chocked.

The step-by-step operator checklist is
`MISSION_STARTUP_PROCESS_APPROVAL_A_20261029.txt`. It keeps later moving tests
and the known-mission rerun behind separate approvals.

