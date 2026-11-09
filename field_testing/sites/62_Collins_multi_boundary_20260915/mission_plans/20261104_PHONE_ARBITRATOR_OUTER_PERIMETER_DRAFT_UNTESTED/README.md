# Phone-controlled outer-perimeter production package — field authorized

This package is the first motion-capable integration of the phone control page,
Pure Pursuit proposals, the bounded communications-degraded guard, the RPi
command lease, and the Teensy v3 phone E-stop relay behavior.

Its current approval state is:

- installation of inactive service definitions is approved;
- v3 Teensy flashing and engine-off commissioning are approved;
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
