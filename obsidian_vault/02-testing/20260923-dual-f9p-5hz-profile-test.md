# Dual-F9P guarded 5 Hz moving-base test — 2026-09-23

> [!update] Final session status
> This note records the pre-mission test plan and initial blocking evidence.
> The guarded 5 Hz profile was subsequently applied and retained for a
> completed 19,825-waypoint supervised master mission. See
> [[20260923-master-manual-field-session-summary]] for final measurements,
> decisions, lessons learned, and follow-up actions.

## Initial blocking evidence

The clearer-sky 120-second baseline produced 533 valid Fixed heading frames
and 48 invalid frames (91.738% valid). The invalid frames retained a Fixed
carrier flag but reported `headValid=false`, `relPosValid=false`,
`isMoving=false`, and a 0.0 m baseline. A single such frame stops the current
strict fail-closed controller. At this point in the session, the revised
profile had not yet passed the parked audit reliably.

## Controlled change

`tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py` is read-only
unless `--apply` or `--restore` is supplied. The guarded profile:

- changes both receivers from 10 Hz to 5 Hz;
- keeps the tractor's UART1-to-UART1 link at 115200 baud;
- changes Base-Link UART1 from MSM7 to MSM4 for GPS, GLONASS, Galileo, and
  BeiDou;
- keeps RTCM 1230 and 4072.0 enabled;
- disables RTCM 4072.1;
- preserves Base-Link USB RTCM input;
- preserves Heading UART1 RTCM input, USB UBX, and USB RELPOSNED output;
- keeps Heading USB NMEA disabled;
- writes RAM, battery-backed RAM, and flash only after an exact confirmation;
- saves every key it can change on both receivers before the first write;
- independently reads back both receivers, even if an ACK is missed;
- can restore the saved targeted values.

No stock ArduSimple configuration file and no factory reset are used.

## Safety state

Keep the deck disengaged and handheld in Pause. Do not select Auto during this
test. Stop `rtcm-server.service` before any configuration read or write because
it normally owns both USB serial ports.

## 1. Read-only proposed-diff review

```bash
cd /home/al/tractor2025
sudo systemctl stop rtcm-server.service
sudo python3 tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py
```

Review every displayed current and target value before using `--apply`.

## 2. Guarded apply

```bash
sudo python3 tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py \
  --apply \
  --backup /home/al/dual_f9p_before_5hz_20260923.json
```

The required confirmation is:

```text
APPLY DUAL F9P 5HZ PROFILE
```

Do not continue unless both independent readbacks pass. If either receiver
fails verification, keep `rtcm-server` stopped and restore the backup:

```bash
sudo python3 tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py \
  --restore /home/al/dual_f9p_before_5hz_20260923.json
```

The restore confirmation is:

```text
RESTORE DUAL F9P BACKUP
```

## 3. Restart and reacquire

```bash
sudo systemctl start rtcm-server.service
systemctl is-active rtcm-server.service
```

Require `active`. Remain in Pause under clear sky and wait about one minute for
LED4 to remain green.

## 4. Scored 120-second audit

```bash
cd /home/al/tractor2025 && sudo systemctl stop rtcm-server.service && sudo python3 tractor_rpi/testing/audit_dual_f9p_moving_base_20260916.py --expected-meas-ms 200 --observe-seconds 120 --json-output /home/al/dual_f9p_5hz_clear_sky_20260923.json; rc=$?; sudo systemctl start rtcm-server.service; systemctl is-active rtcm-server.service; echo "Audit exit code: $rc"
```

The open-sky starting gate is at least 95% usable heading, but the target for
mission use is zero or nearly zero poison frames. Any repeated Fixed-carrier,
zero-baseline frame remains a mission blocker even if the audit process exits
zero. Review the complete report before moving or selecting Auto.

## Result after this plan was written

The 5 Hz audit produced 594/598 valid fixed frames (**99.331%**) at 4.982 Hz.
The profile was then evaluated while moving and retained for the completed
master mission. Runtime policy was explicitly changed to the September 18
basic gate while strict preflight/start checks remained. This was a supervised
risk decision, not a claim that every heading frame was perfect.
