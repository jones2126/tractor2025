# 2026-10-05 Field Stop and Recovery Analysis

## Conclusion

The recorded stops were caused by loss of the phone-control network path while
the router/Starlink battery was near 11.7 V. They were not caused by a steering,
JRK, GPS-position, or heading fault.

The recovery archive recorded:

- 14 phone-heartbeat expirations: 9 while phone Auto was selected and 5 while
  phone Manual was selected;
- request round-trip time as high as 28.8 seconds and multi-second command
  gaps;
- simultaneous ZeroTier/NAS and RTCM routing failures;
- normal steering and JRK diagnostics and a final stopped Pause state; and
- the post-expiry safety latch working as designed: each expiry sent a stop
  burst, latched Pause, invalidated the session, and required a new guarded
  mode selection.

The logs also exposed a software ambiguity. The Wi-Fi firmware reports the
same low-level steering mode in phone Manual and phone Auto, while the bridge
publishes the authoritative phone mode separately as `wifi_control.mode`.
The mission controller and dashboard previously used the ambiguous low-level
mode. That could let Manual movement update route state and could label Manual
as Auto even though the Teensy still gave Manual physical priority.

## Safety corrections

1. The mission controller now uses `wifi_control.mode` when available and only
   falls back to the old steering mode for non-Wi-Fi firmware.
2. The dashboard displays `AUTO ACTIVE` only when the authoritative phone mode
   is Auto and all safety inputs are ready. Phone Manual displays
   `MANUAL ACTIVE — MISSION HELD`.
3. The controller requires five continuous seconds of fresh Wi-Fi heartbeat
   before it permits Auto navigation.
4. The dashboard resets its link timer after a stale sample and displays
   `NETWORK UNSTABLE — REMAIN IN PAUSE` during a ten-second recovery lockout.
5. The phone server accepts motion claims and commands only from
   `192.168.10.0/24`, the tractor router LAN. ZeroTier remains available for
   read-only status and maintenance but cannot carry the motion heartbeat.
6. Mission start now requires an explicit confirmation that router power is
   regulated, above the field minimum under load, and protected by an active
   low-voltage alarm.

## Router-power requirement before another field run

Software cannot measure the present external router/Starlink battery. Before
another moving test, install and verify:

- a regulated supply sized for the router and Starlink peak load;
- a visible voltmeter or low-voltage alarm at the loaded battery/supply input;
- a documented abort voltage established from the battery chemistry, power
  converter specification, and a loaded test; and
- enough reserve capacity for the complete test plus recovery time.

The observed 11.7 V failure point is evidence that the prior supply was not
acceptable, not a proposed operating threshold. Do not choose a final cutoff
from open-circuit voltage alone.

