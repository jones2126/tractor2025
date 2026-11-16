# Phone-controlled outer-perimeter production package — field authorized

This package is the first motion-capable integration of the phone control page,
Pure Pursuit proposals, the bounded communications-degraded guard, the RPi
command lease, and the consolidated Teensy v5 phone E-stop relay behavior.

Its current approval state is:

- installation of inactive service definitions is approved;
- consolidated v5 Teensy flashing and engine-off commissioning are approved;
- production boot activation is approved after the motor-off Manual test;
- the exact 1.00 m/s blades-off perimeter mission is field approved; and
- the production services replace the incompatible legacy boot services.

The phone replaces the retired NRF handheld for Manual, Auto, Pause, and
E-stop. The large red tractor E-stop still grounds the ignition independently.
The rear transmission lever remains a separate mechanical movement barrier.

For the prepared production startup, the phone connects to the GL router
(gateway `192.168.8.1`) and ntfy opens the Tractor01 local-control server at
`https://192.168.8.20:8765/`. ZeroTier remains available for remote status but
is not sent as the handheld control link.

The exact reviewed outer-perimeter route remains unchanged: 3,462 waypoints,
689.949 m, 1.00 m/s, clockwise, with LF-normalized SHA-256
`b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64`.

Run the verifier and software suite from the repository root:

```bash
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/verify_package_20261104.py
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/run_software_tests_20261104.sh
```

The motor-off Manual test, production reboot, local-only notification test,
and Manual repositioning to the reviewed start have passed. Field motion is
authorized only through `mission_dashboard_production_20261109.py`, with the
blades off and direct supervision maintained for the entire perimeter run.

## Prepared automatic post-run evidence collection

The launcher now prepares one run ID and exact paths for the synchronized field
CSV, Pure Pursuit CSV, dashboard transcript, and JSON run manifest. On normal
completion or dashboard `Ctrl+C`, its exit handler first stops the field logger
and then invokes
[`collect_tractor01_run_evidence_20261114.py`](../../../../tools/collect_tractor01_run_evidence_20261114.py).
The dashboard waits for that handler to finish and prints either
`EVIDENCE COLLECTION PASSED` with the `.tgz` and `.tgz.sha256` paths, or a
prominent failure that requires manual collection before shutdown.

The collector is non-destructive. It retains the source logs, copies the exact
run files, current phone/recovery evidence, current production service and
network diagnostics, repository state, runtime phone diagnostics, and the
applicable `/var/log/pilog` files. It verifies an internal `SHA256SUMS`, creates
the compressed archive, reopens the archive for validation, and writes and
checks the external SHA-256 value.

This automatic path is being synchronized to Tractor01 with the 2026-11-16
revision but remains **UNVALIDATED ON TRACTOR01** until exercised in a
separately approved stationary test. Copying the archive
and checksum off Tractor01 and independently verifying the downloaded archive
remain manual operator steps.
