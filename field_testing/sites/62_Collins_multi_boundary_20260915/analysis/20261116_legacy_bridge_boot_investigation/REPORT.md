# Retired Teensy bridge boot investigation — 2026-11-16

Read-only investigation before Al's planned reboot. No installed unit, enablement,
process, hardware setting, or remote repository file was changed.

## Confirmed findings

- Tractor01 clean main: `1b03abda76625d5c7214d87fad6bdbd4071534bf`.
- Pre-reboot boot ID: `f848bc54-967a-41c7-b949-264d7ee7a4bc`.
- `teensy-bridge.service` is disabled but running; it owns `/dev/teensy`
  (`/dev/ttyACM0`), PID 13199 at inspection.
- Its unchanged October 2 drop-in selects
  `tractor_rpi/teensy_serial_bridge_20261002.py`; the underlying service file
  was last modified July 9. It was not re-created by this chat's Git pulls.
- **Enabled** `tractor-startup-preflight.service` still has
  `Requires=rtcm-server.service teensy-bridge.service`. Its boot symlink dates
  to October 26. Disabling the bridge's own boot symlink did not remove this
  dependency, so the preflight can still activate it.
- The journal places the old bridge's first start at **24.409 seconds after
  boot**. It initially failed to bind the not-yet-present network address and
  retried at 30 and 35 seconds. The legacy startup-preflight failed its dependency
  at 26 seconds. The bridge's `Restart=always` remained in effect.
- The current running bridge invocation started at boot-monotonic 1866.634
  seconds (14:26:05 EST after wall-clock adjustment). The journal explicitly
  identifies this as an automatic restart job, not evidence of a manual start.
- Both production arbitrators are enabled but inactive, with no start entries
  in the current-boot journal. Installed definitions match the repository copies.
- Production RPi arbitrator declares `Conflicts=teensy-bridge.service`.
  The phone service requires that arbitrator.
- The production boot installer disables the old bridge and Wi-Fi service,
  and enables the two arbitrators, but does **not** migrate or disable
  `tractor-startup-preflight.service`. This leaves contradictory boot requests.
- LED service is disabled but also retains a `Wants=teensy-bridge.service`.
  Disabled/static historical Wi-Fi services retain similar dependencies.
  No additional matching cron/device-rule trigger was found in the inspected
  `/etc/cron.d`, `/etc/crontab`, `/etc/rc.local`, and `/etc/udev/rules.d` paths.

## Conclusion and limits

The bridge was incompletely retired: an enabled legacy startup-preflight still
pulls it into boot. This is a confirmed configuration defect and directly
explains why "disabled" did not prevent activation. Conflict with the new
arbitrator is the likely explanation for the production pair not starting.
The retained journal does not show which boot transaction jobs systemd rejected;
the exact arbitration of those conflicting requests remains an inference.

Early wall-clock dates are inconsistent with current uptime because the clock
changed; boot ID and monotonic timestamps establish ordering. Do not infer that
the current boot actually began November 11 from the early wall-clock labels.

## Proposed later repair — not performed or authorized by this report

Reconcile the startup-preflight unit and script with the current production
arbitrators and canonical preflight, or explicitly retire that legacy automation.
Review other legacy dependency edges, preserve any intended startup notices,
and add boot-configuration checks. Masking the old bridge alone would leave
the old startup-preflight dependency broken. Starting the new pair manually
would not repair boot configuration. Service changes and any motion remain
separate decisions.

## Reboot comparison

Preserve engine-off, blades-off stationary conditions. Al will initiate reboot.
After reconnection compare boot ID, enabled/active states, current-boot service
journals, dependency results and `/dev/teensy` ownership before starting anything.
Do not assume reboot resolves contradictory installed dependencies.

Evidence: `pre_reboot_snapshot.txt` contains captured installed definitions,
enablement, monotonic start records, boot ID and relevant logs. It is a diagnostic
snapshot, not a complete journal archive.

## Reboot comparison captured 15:04:46 EST

New boot ID: `8b7b5cab-1d22-47de-b784-bc594d4920e8`; clean repository remained
at `1b03abd`. The bad state recurred without any service changes by this chat:

- Disabled legacy bridge running and owning Teensy, PID 1837.
- Enabled production RPi and phone arbitrators both inactive/dead, with no
  current-boot start entries.
- RTCM server active/running.
- Enabled legacy startup-preflight stuck waiting for its qualified network path.
- Bridge first started at 31.096 seconds, failed address binding, retried, and
  reached its running invocation at 44.577 seconds after boot.

The repository startup script selected by the installed preflight unit additionally
requires **retired** `tractor-wifi-control.service` to be active before it attempts
any preflight notification. It defaults to the October preflight and obsolete
`teensy_main_20261003_wifi_v3` identity. Thus this automation needs a full migration,
not just a replacement bridge dependency. The missing phone-link notice is
consistent with the production phone server never starting; no phone-server
notification delivery attempt is established by these logs. ZeroTier reachability
alone does not satisfy this legacy gate and does not establish phone control.

See `post_reboot_snapshot.txt`. Installed services remain unchanged pending Al's
decision on the repair.

## Prepared follow-up — not installed

Al approved preparation and testing only. The candidate migration and persistent
legacy bridge mask are documented in `tractor_rpi/setup/STARTUP_REPAIR_20261116.md`
from the repository root. Preparation changed only repository files; it did not
install, stop, start, enable, disable or mask a live Tractor01 service. The new
root installer mutation path and boot acceptance remain separately gated.
