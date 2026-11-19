# Support request: dual ZED-F9P moving-base heading repeatedly remains RTK Float despite fresh corrections and verified settings

Hello ArduSimple Support,

I need help diagnosing unreliable acquisition/reacquisition of RTK Fixed position
and moving-base heading with two ArduSimple ZED-F9P receivers on a tractor. On
November 18, 2026, the tractor remained in the same location and orientation all
day. We repeatedly observed prolonged Float solutions, interspersed with fully
Fixed solutions, without moving the tractor or changing the receiver profile.
This prevents dependable startup and field testing.

I would like a specific diagnostic procedure and a review of our configuration.
We have not isolated the cause to receiver hardware, firmware, configuration,
correction content, RF conditions or host software.

## Hardware and data flow

- Two ArduSimple ZED-F9P boards, rear Base-Link and front Heading, mounted at a
  fixed antenna spacing of approximately 1.1 m. Earlier physical measurements
  were approximately 1.06–1.10 m; today's Fixed reports were around 1.14 m. We
  have not independently re-surveyed the spacing today.
- Last recorded receiver identification, from September 16: **HPG 1.32,
  PROTVER 27.31, MOD ZED-F9P** on both receivers. This has not been re-read today.
- A Raspberry Pi 5 connects to both receivers by separate USB serial devices.
- External fixed-base RTCM arrives over TCP through ZeroTier and is forwarded
  into **Base-Link USB**.
- **Base-Link UART1 TX → Heading UART1 RX**, 115200 baud, carries the moving-base
  reference/observations. The Pi reads Heading UBX-NAV-RELPOSNED over USB.
- Board revisions, antenna models, wiring photographs and a current MON-VER
  capture can be supplied if needed; please identify which details you require.

## Symptoms and representative measurements

Position sometimes becomes RTK Fixed while Heading remains Float for many minutes.
At other times both are Float. Waiting 60, 120 and 300 seconds has not reliably
resolved it. Restarting the host GPS service has sometimes preceded recovery,
but has also failed to help. We have not established causation.

A representative failed sample on November 18 showed:

| Measurement | Result |
|---|---|
| Base-Link position | RTK Fixed in some failures; RTK Float in others |
| Heading carrier | Float in every observed sample of the final window |
| Heading satellites used | Approximately 29–31, using NAV-PVT/NAV-SAT diagnostics |
| Base-Link satellites used | Approximately 30–31 |
| Heading satellites visible | Approximately 45–47 in several samples |
| Mean C/N0 | Heading approximately 37–41 dB-Hz; Base-Link approximately 35–37 dB-Hz |
| Base-Link HDOP | Approximately 0.58–0.64 in several later samples |
| Differential correction age | Typically approximately 0.6–1.4 seconds |
| External RTCM forwarding | Steady approximately 430–500 bytes/second |
| Heading output | NAV-RELPOSNED counter advances at approximately 5 Hz |
| RELPOSNED flags | gnssFixOK, diffSoln, relPosValid, isMoving and headValid true in 100% of snapshots in the cited windows |
| Missing reference flags | refPosMiss=false and refObsMiss=false throughout those windows |
| Reported heading accuracy | Approximately 0.5–0.7 degrees despite Float |
| Reported baseline in later failures | Approximately 1.11–1.16 m; one earlier Float result was 1.9309 m |

The Pi publishes snapshots at 20 Hz; heading itself updates at approximately 5 Hz.
Fractions above describe snapshots, not 100 independent heading measurements.
We require a Fixed carrier separately from headValid/accuracy/baseline, so Float
does not become a passing result merely because those other fields look plausible.

There were external correction connection interruptions earlier in the day and
during startup. However, later persistent-Float periods also occurred with fresh
correction age and several uninterrupted minutes of forwarding. Byte forwarding
is not a receiver-side RTCM CRC/message-use measurement; we do not yet capture
that distinction directly.

## Configuration verified today

A read-only audit found **all 40 targeted settings matched** the established
profile: 18 Base-Link and 22 Heading values. No settings were changed by that audit.

- Both: CFG-RATE-MEAS=200 ms, CFG-RATE-NAV=1; UART1 enabled at 115200.
- Base-Link: USB RTCM input enabled; UART1 RTCM input disabled and output enabled.
- Base-Link UART1 outputs: **1074, 1084, 1094, 1124, 1230 and 4072.0 enabled**;
  **1077, 1087, 1097, 1127 and 4072.1 disabled**.
- Heading: UART1 RTCM input enabled, UART1 RTCM output disabled, USB UBX output
  enabled, USB NAV-RELPOSNED enabled, USB NMEA protocol and audited NMEA rates disabled.
- Before every normal GPS service start, our startup hook writes the Heading
  profile to **RAM only**, then independently reads back all 22 values. That check
  passes. The startup hook does not reapply the Base-Link settings each time.
- The separate guarded full-profile apply action can write RAM/BBR/flash, but it
  was **not used during today's read-only audit**. No factory reset was performed.

Please review the linked configuration code and attached actual readback rather
than assuming stock configuration files or the older historical 10 Hz profile.

## Controlled comparisons and successful results

Times below are November 18, 2026, US Eastern Standard Time (UTC−5).

| Time | Observation |
|---|---|
| Approximately 12:03 | Original GPS service, after restart: position Fixed, Heading Fixed in 100% of the sample, baseline 1.1472 m |
| Approximately 14:28 | New producer after another normal service restart: position Fixed, Heading Float, baseline 1.1573 m |
| Approximately 14:32 | Temporarily ran original GPS server without reapplying the profile: position and Heading Fixed, baseline 1.1470 m |
| Approximately 14:35 | Temporarily ran new producer without reapplying the profile: position and Heading Fixed, baseline 1.1388 m |
| Approximately 14:37 | Returned to normal new-production service WITH startup write/readback: full preflight passed; position and Heading Fixed, baseline 1.1424 m |
| After clean Pi shutdown/startup | Float recurred; approximately 14:57, both were Float despite fresh corrections, about 31 Heading satellites used and baseline 1.1423 m |
| Approximately 15:12 | After the operator restarted the external RTK BASE service, full preflight passed: position Fixed, Heading Fixed in 100% of snapshots, baseline 1.2335 m, Heading accuracy 0.466 degrees, 31 Heading satellites used, correction age median 1.40 s |

**Important later observation:** the approximately 15:12 recovery followed an
external RTK base service restart, after repeated Tractor01-side restarts had not
reliably recovered Fixed. Please include the fixed-base receiver and Python/TCP
correction-distribution path in the investigation. This is temporal association,
not proof that the base server or receiver is defective. The increased reported
Fixed baseline (1.2335 m versus earlier approximately 1.14 m), with the tractor
unmoved, is also worth explaining; both satisfy our current broad 0.80–1.30 m gate,
but that gate is not an independent baseline accuracy validation.

This shows that both host implementations can receive/report healthy Fixed heading,
and that the startup profile does not invariably cause failure. It does not prove
which event restored Fixed, nor does it exonerate either host implementation.
The final successful run used the normal production startup hook; we did not
remove a requirement or lower a threshold to obtain PASS.

Our independent IMU/wheel-encoder work exposed unrelated host readiness bugs and a
left-encoder startup stall. Those were investigated separately. Neither IMU nor
encoder estimates are substituted into the GNSS values reported above. The tractor
has not moved during these comparisons. The IMU was repositioned separately; it
is not the source of the F9P heading/carrier status.

## Exact code and configuration links

All links are pinned to the repository revision currently pulled to Tractor01,
**06e2fbe32dba7bc995e46a7331e6c2ec274eaf64**, not a moving main branch:

- [Current sensor/GPS producer: readers, RTCM forwarding and publication](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/calc_publish_position.py)
- [GNSS wire decoding, including NAV-RELPOSNED carrier/flags](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/position_gnss_protocol.py)
- [GNSS state updates and separate observational pose model](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/position_model.py)
- [Dual-F9P configuration utility: exact settings, read-only audit, guarded full apply and Heading RAM startup action](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py)
- [Installed position service definition and Heading ExecStartPre](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/setup/calc-publish-position.service)
- [Preflight predicates and receiver-health reporting](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/tractor_preflight.py)
- [Original GPS server used in the comparison](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/tractor_rpi/rtcm_server_20260828.py)
- [External RTK base server observed running today](https://github.com/jones2126/tractor2025/blob/06e2fbe32dba7bc995e46a7331e6c2ec274eaf64/RTKBase/Bridgeville/rtcm_base_server_20260628.py)

## Specific help requested

1. Is this 5 Hz UART1 moving-base message/profile combination appropriate for
   HPG 1.32? Are additional settings or a recommended firmware revision needed?
2. Should the Heading RAM profile be rewritten on every service start, or should
   we verify first and write only differences? Can rewriting unchanged rate/UART
   keys disturb ambiguity resolution? What reacquisition behavior is expected?
3. Which receiver messages and fields should we capture to distinguish correction
   reception/use/CRC problems, insufficient common usable carrier signals,
   interference/multipath, antenna faults and receiver-state problems?
4. Please provide the minimum logging recipe, recommended observation duration
   and configuration/export files you need from BOTH receivers and the fixed base.
   We can collect raw UBX or use u-center with a procedure that preserves the current setup.
5. What documented recovery/reset sequence is appropriate for this persistent-Float
   condition, and what should be preserved before attempting it?

Suggested initial attachments: the actual 40-setting readback
`dual_f9p_profile_review_20261118.txt`, the successful original-service report
`rtcm_post_restart_preflight_20261118.txt`, and the dated failed/successful GPS
preflight reports from the same day. Full raw receiver diagnostics have not yet
been collected. Phone-control URLs/keys are not needed for this investigation.

Thank you,
Al
