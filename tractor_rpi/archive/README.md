# Tractor RPi archived runtime scripts

Inventory updated November 20, 2026.

These folders retain superseded, date-versioned runtime scripts for historical
reproduction and comparison. Current runtime scripts remain directly under
`tractor_rpi/` so the active service and field commands are easy to identify.

- `field_test_loggers/` — field logger versions superseded by
  `field_test_logger_20260828.py`
- `rtcm_servers/` — Tractor01 dual-F9P servers superseded by the canonical
  `tractor_rpi/calc_publish_position.py`, including the August 28 server.
- `pose_producer_20261117/` — pre-consolidation candidate launcher/model/decoder.
- `service_tools/` — helper scripts for retired service names, retained as history.

The separate Tractor02 X20D implementation is under `../variants/tractor02_x20d/`;
it is a hardware variant rather than an old Tractor01 version.

See [November 20 relocation details](RELOCATION_20261120.md) and the machine-readable
`relocation_20261120.json`. Original hashes describe the pre-move source bytes;
some moved files needed import/root-path fixes after relocation. Historical
capture manifests are retained with their original paths/hashes.
