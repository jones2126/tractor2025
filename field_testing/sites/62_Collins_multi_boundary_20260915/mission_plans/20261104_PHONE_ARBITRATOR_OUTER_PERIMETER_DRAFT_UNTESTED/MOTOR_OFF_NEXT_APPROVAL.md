# Next approval: installation and motor-off commissioning

This stage is not yet approved. When separately approved, it will:

1. verify the exact Git revision and package;
2. build the v3 Teensy firmware;
3. stop the current phone and bridge services while the engine is off, blades
   are off, the rear transmission lever is OUT, physical red E-stop is ready,
   and wheels are chocked;
4. flash v3 with the existing rollback image retained;
5. install but not boot-enable the two candidate services;
6. start the arbitrator and phone service in Pause;
7. verify phone Pause, phone E-stop relay closure, guarded E-stop reset to
   Pause, Pi lease expiry, steering fault handling, and service-loss Pause;
8. exercise Manual and Auto intent only with the rear transmission lever OUT;
9. collect and checksum evidence; and
10. restore the known production firmware and services unless a later approval
    explicitly retains the candidate.

Engine start and tractor movement are not part of this stage.
