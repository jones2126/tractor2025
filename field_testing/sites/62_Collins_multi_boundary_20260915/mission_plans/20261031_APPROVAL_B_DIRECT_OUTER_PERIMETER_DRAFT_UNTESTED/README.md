# Approval B direct-router outer-perimeter package

**DRAFT - UNTESTED IN THE FIELD**

This is a disabled preparation package for one separately authorized,
directly supervised, blades-off Approval B run. It does not authorize motion.

## Frozen route

- Existing reviewed outer-perimeter route; no geometry was changed.
- 3,462 waypoints; 689.949 m; approximately 11.50 minutes at 1.00 m/s.
- Clockwise; left deck edge follows the perimeter; 2.00 m lookahead.
- Mission SHA-256 (LF-normalized):
  `b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64`.
- Audit SHA-256 (LF-normalized):
  `54072d526370447f4efd22a84052798543135fc45298e6bda80e983178f5cf25`.

The source package remains at
`../20260928_outer_perimeter_field_test/`. This package references it rather
than copying or silently revising the route.

## Control path

The prepared live path is phone `192.168.8.x` on SSID `GL-SFT1200-9f4` to
Tractor01 `eth0` address `192.168.8.20`. It uses the new named policy
`onboard-router-mission`. The dedicated service has no boot target and
conflicts with both normal ZeroTier control and Manual recovery.

This does **not** alter the 500 ms freshness behavior. COMMUNICATIONS DEGRADED
remains disabled. Pure Pursuit, bridge, Teensy, physical Pause, and e-stop
behavior remain immediate and authoritative.

## What can be done before field authorization

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED/verify_approval_b_package_20261030.py
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED/run_approval_b_outer_perimeter_20261031.sh --verify-only
```

Both commands are read-only. The launcher contains
`APPROVED_FOR_FIELD=false` and refuses live operation. A later, explicit field
authorization must create a new enabled revision after test-day conditions and
the stationary hold points are reviewed.

The post-run collector is `collect_approval_b_evidence_20261031.sh`. It requires
an explicit `--since` time, copies rather than deletes source logs, captures
service/network/repository state, and writes internal plus archive SHA-256
checksums. The GL-router export remains a manual companion artifact because no
safe automated router collection path has been established.

Use `FIELD_CHECKLIST_20261031.txt` tomorrow. Stop at its field-authorization
hold point and report the results before any movement.
