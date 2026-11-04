# Phone-arbitrator outer-perimeter production package — disabled draft

This package is the first motion-capable integration of the phone control page,
Pure Pursuit proposals, the bounded communications-degraded guard, the RPi
command lease, and the Teensy v3 phone E-stop relay behavior.

It is deliberately disabled:

- installation is not approved;
- Teensy flashing is not approved;
- field motion is not approved; and
- neither service has a boot target.

The phone replaces the retired NRF handheld for Manual, Auto, Pause, and
E-stop. The large red tractor E-stop still grounds the ignition independently.
The rear transmission lever remains a separate mechanical movement barrier.

The exact reviewed outer-perimeter route remains unchanged: 3,462 waypoints,
689.949 m, 1.00 m/s, clockwise, with LF-normalized SHA-256
`b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64`.

Run the verifier and software suite from the repository root:

```bash
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/verify_package_20261104.py
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/run_software_tests_20261104.sh
```

The next separate approval is for installation plus a new motor-off test of
the v3 E-stop/reset behavior and motion-capable bridge while the rear
transmission lever remains OUT. It is not approval to start the engine or move.
See `MOTOR_OFF_NEXT_APPROVAL.md` for that stage's exact scope.
