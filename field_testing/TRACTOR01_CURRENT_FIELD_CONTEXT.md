# Tractor01 current field context

Last updated: 2026-11-11

This is durable operational context for assistants helping Al with Tractor01.
Read it before proposing field commands. It records established facts and
required command paths; it does not authorize motion, blades, firmware changes,
service changes, or a mission.

## Command-path rule

PuTTY normally opens in `/home/al`, while the Tractor01 repository is:

```text
/home/al/tractor2025
```

Never assume the shell is already inside the repository. Every proposed
repository command must either:

1. begin with `cd /home/al/tractor2025`; or
2. use an absolute path below `/home/al/tractor2025`.

The canonical manual phone-arbitrator preflight is therefore:

```bash
cd /home/al/tractor2025
sudo python3 tractor_rpi/testing/mission_preflight_phone_control_20261109.py \
  --expected-firmware teensy_main_20261110_rpi_command_lease_v5
```

Require the exact final line:

```text
MISSION PREFLIGHT PASS
```

Any FAIL means remain in Pause and do not select Auto.

## Current production control stack

- Canonical Tractor01 checkout: `/home/al/tractor2025`
- Teensy firmware identity:
  `teensy_main_20261110_rpi_command_lease_v5`
- Commissioned steering calibration: right `191`, center `525`, left `885`
- Production services:
  - `rtcm-server.service`
  - `tractor-rpi-arbitrator.service`
  - `tractor-phone-arbitrator.service`
- Phone control is local to the onboard GL router at Tractor01 address
  `192.168.8.20`, port `8765`.
- Mission-dashboard access is expected over Tractor01 ZeroTier address
  `192.168.193.76`, normally port `8088`.
- A dashboard host of `0.0.0.0` is the production default listen setting. It
  means listen on all Tractor01 IPv4 interfaces; the operator still opens the
  concrete ZeroTier URL `http://192.168.193.76:8088/?key=...`.

## Heading-F9P initialization and preflight

The installed `rtcm-server.service` owns the Heading-F9P device. Its
`ExecStartPre` runs:

```text
configure_dual_f9p_5hz_profile_20260923.py --heading-startup
```

That hook applies the established moving-base profile in volatile RAM and
performs an independent readback on each successful service start, including a
successful start during the current boot. The phone-arbitrator preflight checks
for the installed hook and successful current-boot proof.

Do not stop or restart `rtcm-server` merely to rediscover facts when the current
boot already contains successful independent-readback proof and the full
preflight passes. If proof is missing or preflight reports a Heading startup
failure, remain stationary in Pause and diagnose before any movement.

Required mission-start invariant remains: stationary in Pause with blades off;
current-session Heading-F9P write/readback proof; `rtcm-server` active; heading
reacquired; full `MISSION PREFLIGHT PASS`.

## Raspberry Pi throttling monitoring

Operator-confirmed live Tractor01 behavior as of 2026-11-11:

- Boot produces an ntfy notice containing `vcgencmd get_throttled` state.
- A healthy current boot reports `throttled=0x0`.
- If a power/throttling problem develops during operation, repeated ntfy
  notices are sent.
- Al reported that this state is also retained in operational telemetry.

Do not ask Al to rerun `vcgencmd get_throttled` merely to discover a value he
has already supplied for the current boot. It remains legitimate as an explicit
preflight gate or when diagnosing a new power alert.

The exact installed notifier/telemetry implementation was not found in GitHub
`main` during the 2026-11-11 review. Treat the behavior above as verified
live-system context, but do not invent a repository service or field name.
Trace the installed unit/script on Tractor01 before changing or documenting its
implementation details.

## Dashboard interpretation before controller start

The review dashboard intentionally cannot start Pure Pursuit. Before the
mission controller exists:

- `Last fresh controller telemetry: never` is expected.
- `Recovery lockout: waiting for stable control` is expected in the review
  dashboard because its legacy stability calculation requires controller
  telemetry that the disabled dashboard never starts.
- Those two labels alone do not indicate a phone, Teensy, GPS, or actuator
  fault.

Live bridge values must still show Pause, zero steering PWM, neutral
transmission, fresh RTK Fixed position, fixed/valid heading, acceptable heading
accuracy and baseline, and no actuator fault.

## Current outer-perimeter package

The current reviewed boundary package is:

```text
field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/
20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED
```

Route identity:

- Stable asset: `62COLLINS-OUTER-PERIMETER-20260928`
- 3,462 waypoints
- 689.949 m
- 1.00 m/s
- clockwise
- route SHA-256:
  `b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64`

Keep review/dashboard inspection, preflight approval, physical movement, and
blade authorization as separate decisions.

## Current 2026-11-11 commissioning evidence

- Firmware v5 compiled and uploaded successfully from GitHub `main` commit
  `755bde9`.
- Production services restarted successfully after reboot.
- Straight steering settled at pot `526` for center target `525`, with zero PWM
  in Pause and no steering fault.
- Full-left test: pot `881`, target `885`, zero PWM, no fault.
- Full-right test: pot `194`, target `191`, zero PWM, no fault.
- JRK supply telemetry: `12.57 V`, valid; handheld multimeter: `12.5 V`.
- Transmission actuator sweep behaved as expected with engine off and the
  transmission physically disengaged.
- Phone E-stop actuated and released the relay audibly.

These stationary commissioning observations do not themselves authorize a
mission or physical movement.
