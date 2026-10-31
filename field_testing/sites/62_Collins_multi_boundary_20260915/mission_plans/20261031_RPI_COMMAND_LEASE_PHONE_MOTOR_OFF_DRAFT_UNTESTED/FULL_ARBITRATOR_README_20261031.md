# RPi full arbitrator — motor-off draft

This extension verifies the decision layer joining explicit phone intent,
fresh Pure Pursuit command and health samples, exact route identity,
GPS/heading and route-corridor health, and fresh Teensy/local-actuator safety
status.

The additive producer integration consists of:

- `wifi_rpi_arbitrator_control_20261031.py`, which publishes requested mode
  separately from measured operator-link freshness on local UDP 6014; and
- `pure_pursuit_arbitrator_feed_20261031.py`, which preserves the existing
  controller/dashboard outputs while duplicating a versioned safety record,
  including the normalized exact-route SHA-256, on local UDP 6015.

Neither producer is selected by an installed service in this revision. The
serial-authority consumer remains deliberately absent pending a motor-off
producer-feed test.

It is **not field-motion authorization**. It installs no service, changes no
boot setting, opens no hardware, and does not enable the 30-second/30-meter
state in the production phone service.

The 500 ms local command/Teensy lease remains unchanged. Only an already
running AUTO mission with every other local check healthy can enter the
separate sticky communications-degraded envelope. Reconnects cannot reset its
incident start time or distance. Manual phone loss and every non-phone safety
failure require immediate Pause.

Run from the repository root:

```bash
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_RPI_COMMAND_LEASE_PHONE_MOTOR_OFF_DRAFT_UNTESTED/run_full_arbitrator_software_tests_20261031.sh
```
