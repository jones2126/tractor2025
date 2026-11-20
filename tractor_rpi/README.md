# Tractor RPi production source

Inventory reviewed November 20, 2026. These repository paths describe the current
production design and documented installation. This cleanup did not inspect or
change the running Tractor01 checkout.

The immediate folder holds current entry points and runtime dependencies.
Standalone tools, uninstalled candidates, historical versions and other hardware
variants belong in named subfolders. A code file must retain a date in its
filename or header/docstring; stable production filenames need not change for
each revision. Existing dated dependencies remain valid production code.

## Current entry points and supporting modules

| Files | Role |
|---|---|
| `calc_publish_position.py` | Canonical producer selected by `setup/calc-publish-position.service`; GNSS/IMU/wheel acquisition and raw GNSS publication |
| `position_model.py`, `position_gnss_protocol.py`, `imu_sensor.py` | Current decoder, sensor helpers and shadow pose model |
| `telemetry_recorder.py`, `telemetry_ipc.py`, `field_telemetry_schema.py` | Continuous recorder, local IPC and field-row mapping |
| `tractor_preflight.py` | Canonical recorder-querying readiness/preflight |
| `teensy_serial_bridge_rpi_arbitrator_20261104.py` | Current tractor-Teensy owner and RPi safety selection |
| `teensy_serial_bridge_rpi_arbitrator_motor_off_20261031.py`, `teensy_serial_bridge_rpi_lease_pause_20261031.py`, `teensy_serial_bridge_20261002.py`, `teensy_serial_bridge_20260728.py` | Required inherited bridge/parser modules; these are not four independently running bridge services |
| `rpi_arbitrator_inputs_20261031.py`, `rpi_arbitrator_wire_20261031.py`, `rpi_command_lease_adapter_20261031.py`, `rpi_command_lease_protocol_20261031.py`, `rpi_command_lease_runtime_20261031.py`, `rpi_safety_supervisor_20261031.py` | Current arbitration/command-lease dependencies |
| `communications_degraded_guard_20261029.py`, `phone_auto_recovery_config_20261022.py` | Retained navigation/phone package runtime dependencies |
| `led_status_controller.py` | LED tower implementation referenced by repository service configuration; not a claim its installed unit is running |

The local TLS phone entry point remains under `testing/webrtc/` because the
commissioned frozen package references it there. Folder names alone do not prove
installation state. Migrating that package is separate from this cleanup.

## Tools, experiments, history and variants

| Location | Contents |
|---|---|
| `tools/export_telemetry_run.py` | Verify closed segments and export explicit windows: `python -m tractor_rpi.tools.export_telemetry_run --help` |
| `testing/` | Tests, experiments and established frozen-package entry points |
| `testing/battery_voltage/battery_voltage_logger_20261111.py` | Uninstalled independent battery-logger candidate, not the continuous recorder |
| `archive/pose_producer_20261117/` | November 17 launcher, implementation and model/protocol superseded by the consolidated producer |
| `archive/rtcm_servers/` | Superseded dual-F9P RTCM servers, including `rtcm_server_20260828.py` |
| `archive/service_tools/check_services.sh` | Historical helper listing retired services; do not use its printed restart commands for the current stack |
| `variants/tractor02_x20d/` | Separate X20D hardware implementation, not the Tractor01 dual-F9P producer |
| `setup/` | Current service/install configuration and historical setup references; consult `CONSOLIDATED_STARTUP_REVIEW.md` and AGENTS.md before choosing an installer |

Temporary root-level exception: `field_test_logger_20260828.py` is referenced by
frozen mission launchers and verifiers. It remains solely for that compatibility
boundary; it must not compete with the boot recorder for UDP 6009. Moving it needs
a separate reviewed package migration, not silent changes to pinned launchers.

The archived candidate can still be inspected without hardware:

```powershell
python tractor_rpi/archive/pose_producer_20261117/calc-publish-position-20261117.py --help
```

Current services/imports continue to use `tractor_rpi.calc_publish_position`,
`tractor_rpi.position_model` and `tractor_rpi.position_gnss_protocol`. No root-level
forwarding copies of the archived candidate remain.

See [the relocation record](archive/RELOCATION_20261120.md) for exact moves,
original hashes, necessary path edits and verification.
