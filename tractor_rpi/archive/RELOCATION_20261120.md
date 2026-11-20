# RPi source-folder cleanup — November 20, 2026

Requested by Al to keep production source folders focused on current production
code and preserve visible date references. These are local repository changes;
no Tractor01 connection, service restart, installation or firmware change occurred.

## Files moved

Paths below are relative to `tractor_rpi/`.

| Former root file | New location | Reason |
|---|---|---|
| `calc-publish-position-20261117.py` | `archive/pose_producer_20261117/calc-publish-position-20261117.py` | Historical candidate launcher |
| `calc_publish_position_20261117.py` | `archive/pose_producer_20261117/calc_publish_position_20261117.py` | Historical candidate implementation |
| `position_model_20261117.py` | `archive/pose_producer_20261117/position_model_20261117.py` | Historical candidate model |
| `position_gnss_protocol_20261117.py` | `archive/pose_producer_20261117/position_gnss_protocol_20261117.py` | Historical candidate decoder |
| `rtcm_server_20260828.py` | `archive/rtcm_servers/rtcm_server_20260828.py` | Dual-F9P producer superseded by the consolidated producer |
| `rtcm_server_x20d_20260702.py` | `variants/tractor02_x20d/rtcm_server_x20d_20260702.py` | Another hardware variant, not the current Tractor01 producer |
| `battery_voltage_logger_20261111.py` | `testing/battery_voltage/battery_voltage_logger_20261111.py` | Uninstalled independent logger candidate |
| `check_services.sh` | `archive/service_tools/check_services.sh` | Lists retired service names; historical helper |
| `export_telemetry_run.py` | `tools/export_telemetry_run.py` | Standalone evidence-verification/export utility |

`relocation_20261120.json` records original and relocated SHA-256 values and whether
the bytes changed. Original hashes preserve the exact pre-move source identity;
they are not replaced by the post-move hashes. Imports/root calculations in the
archived pose candidate needed location edits; tools received dated placement
comments. Files which needed no edits retain their bytes.

## Necessary reference changes

- Archived pose imports now use `tractor_rpi.archive.pose_producer_20261117`;
  its launcher and capture-source metadata resolve the repository root at the new
  depth. Its neighboring source-hash references still resolve within the group.
- Export consumers now import `tractor_rpi.tools.export_telemetry_run`, including
  consolidated-startup tests, the November 19 replay builder and the new encoder
  analysis/extraction scripts. Source-manifest generation uses the new tool path.
- Historical parser tests read the August 28 RTCM server at its archive path.
- The uninstalled battery candidate's test, draft service and installer reference
  the `testing/battery_voltage/` module. This does not install that draft service.
- Historical RTCM unit templates and the old installer reference the archived
  RTCM file. The old installer is explicitly labelled historical and remains
  unsuitable for installing the current consolidated stack.
- The old candidate drop-in references the archived candidate launcher. Its
  historical guide now prominently identifies the current versus archived paths.
- AGENTS.md and the folder inventory identify canonical code, subfolder roles and
  the temporary frozen-package exception. Historical capture files and pinned
  mission packages were not rewritten. Old paths in historical reports/manifests
  remain historical evidence; use this table to locate the corresponding source.

## Files intentionally retained at the production level

The commissioned bridge imports older bridge/parser implementations through an
inheritance chain. Those dated files and the command-lease/arbitrator modules are
current runtime dependencies, not removable leftovers. Their locations and code
were preserved.

`field_test_logger_20260828.py` is a documented temporary exception: many frozen
mission launchers and package checks require its exact root path. Relocating it
would require a separate package migration and hash review. It is not part of the
boot-recorder path and must not run against the continuous recorder's UDP 6009.

The canonical `calc_publish_position.py`, `position_model.py` and
`position_gnss_protocol.py` retain their service/import names. Nine undated current
production Python modules received a dated placement-review comment. AST comparison
against the pre-cleanup Git version confirms their executable syntax is unchanged;
the relocated export tool also has an unchanged executable syntax tree. A review
date does not claim new field validation.

The commissioned phone package remains under its established `testing/webrtc/`
path; moving a live frozen package is outside this root-folder cleanup.

## Verification

The following hardware-free tests passed after the moves:

```powershell
python -m unittest tractor_rpi.testing.test_calc_publish_position_20261117 tractor_rpi.testing.test_consolidated_startup tractor_rpi.testing.test_battery_voltage_logger_20261111 tractor_rpi.testing.test_gps_satellite_logging_20260828 tractor_rpi.testing.test_startup_preflight_service_20261026
```

86 tests passed. Current producer, archived candidate launcher, relocated export
tool and draft battery module also passed `--help` startup checks without opening
hardware. Production-module AST comparisons, missing-old-path/present-new-path
checks and relocation hash checks passed. The existing seven encoder-analysis
mechanics tests are also retained and run against the updated export import.

No files were deleted as obsolete; the nine original paths are replaced by the
new locations. The move record and Git history preserve the previous layout.
Changes must be committed and synchronized through GitHub before a separately
reviewed update to another checkout. Installed units on Tractor01 were not edited.
