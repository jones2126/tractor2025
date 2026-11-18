# Future improvement: optional RTCM restart after preflight GPS failure

Recorded November 18, 2026 at Al's request. Proposal only; no implementation or
change to code, shell files, installed services, or current preflight behavior.

## Requested behavior

Add an operator-selectable restart/retry option to preflight when an applicable
GPS/heading failure occurs. The recovery action would restart the existing
`rtcm-server.service`, which reapplies and independently verifies the established
Heading F9P RAM profile and reconnects external corrections, then run a complete
fresh preflight. A restart must never itself count as a passing result.

Keep this optional and bounded to one recovery attempt, with the operator
confirming the tractor is stopped in Pause, engine off and blades disengaged.
Do not restart for unrelated failures or create an unattended restart loop.
Preserve timestamped failure diagnostics before restarting and retain the
post-restart report. Keep normal motion/watchdog gates unchanged. Design the
interactive choice and any explicit command-line option before implementation;
unattended startup preflight must not wait indefinitely for an interactive reply.

## November 18 field evidence

- Heading configuration/readback already passed before recovery.
- Initial heading remained Float with a 1.9309 m baseline despite adequate
  satellite counts. Subsequently both position and heading remained Float
  through a five-minute preflight wait; final baseline was 1.1980 m.
- Corrections were fresh in the failed sample (median age 1.20 s); Heading used
  29 satellites and reported reference data present. Earlier service journals
  showed network/connection interruptions followed by sustained forwarding.
- After one operator-executed `rtcm-server.service` restart, the full preflight
  passed: position RTK Fixed, heading carrier Fixed in 100% of samples,
  baseline 1.1472 m, heading accuracy 0.514 degrees, 29 Heading satellites,
  actual Pause, neutral 2836 and commissioned v5 firmware.
- This demonstrates recovery following restart, not a proven root cause.
  Reapplying Heading configuration, reconnecting corrections and coincident
  convergence were not isolated experimentally.

Source reports supplied by Al, retained outside the repository at
`C:/Users/al532/Downloads/20261118/`:

- `rtcm_diagnostics_20261118.txt`
- `rtcm_post_restart_preflight_20261118.txt`

Current operating context: `field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`.
The new combined position producer was not activated during this recovery.
