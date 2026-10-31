# RPi command-lease change boundary — 2026-10-31

## Purpose

Separate phone/network policy from the Teensy without changing the field-tested
control system in place. This work starts from commit
`3e0b012cb470542735431a6adb4ba9a78bbd6659` on the isolated branch
`codex/rpi-command-lease`.

This document and its automated guard are containment measures. They are not
field authorization and do not enable motion.

## Responsibility boundary

The RPi owns:

- phone session and freshness;
- local-network and ZeroTier observations;
- Pure Pursuit freshness and mission state;
- the bounded, sticky COMMUNICATIONS DEGRADED time-and-distance envelope;
- the decision to issue Pause, Manual, or Auto commands to the Teensy.

The Teensy owns:

- a generic RPi command-lease watchdog, independent of Wi-Fi terminology;
- physical Pause and e-stop authority;
- serial-frame validation and command-range validation;
- transmission/JRK safety;
- steering response and steering-fault latching;
- immediate neutral/hold behavior when the RPi lease or a local safety check
  fails.

Phone freshness must not be evaluated by new Teensy logic. A phone reconnect
must not reset an active RPi degradation incident. The existing 500 ms Teensy
watchdog is not extended to 30 seconds.

The generic serial protocol additionally requires:

- an RPi session identifier and monotonically increasing sequence number;
- a new session to claim authority with a zero Pause frame while physical
  Pause is active;
- any command-lease expiry to latch Pause locally until a fresh explicit RPi
  Pause frame is accepted;
- matching `RPI_CTL` acknowledgement of that Pause before the RPi adapter may
  emit Manual or Auto motion frames.

These rules prevent an RPi process restart or delayed serial backlog from
silently re-arming motion.

## Frozen reference paths

The first implementation phases may not edit these proven files:

| Path | SHA-256 at the baseline |
|---|---|
| `tractor_teensy/src/archive/teensy_main_20260804.cpp` | `9588bdb5d37954c6aac610d50440193349dae637e6a9a6c1097fe9783b450b76` |
| `tractor_teensy/src/teensy_main_20260914.cpp` | `9809a2a5a749cfa843551bdf0d08e8d52077656184665694ac9ae6c53085b801` |
| `tractor_teensy/src/teensy_main_20261003_wifi.cpp` | `0fcfed36f8f293f4864f13e047dab54def0f6c871b597321de8414e85cd61ee2` |
| `tractor_rpi/teensy_serial_bridge_20260728.py` | `0a6ddb487689f6fbcbec23823fd5ee95b2e771cd3800507dcad9640ba555882e` |
| `tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py` | `f4f43b24c0b8c7529c10a8335d829d4ae22ec00ed01b56ba074a7f4b8c54c58b` |
| `tractor_rpi/testing/webrtc/wifi_primary_control_20261022.py` | `9b3f7b1bb8b65dd4fafbd32ed645126ca5a1ec4f30d4b1a55878465b085eab15` |
| `tractor_rpi/communications_degraded_guard_20261029.py` | `b89aae435f5ac8592770e82d92d2239839dd6890ee4c7f1455d4b352afa67d1d` |

The guard also forbids changes to:

- `tractor_teensy/platformio.ini` and `platformio.wifi-primary.ini`;
- the entire disabled Approval B field package;
- existing system-service installers and unit definitions.

## Allowed initial changes

Only additive, opt-in work is allowed:

- new design records and automated tests;
- a new RPi supervisor module with no boot or service integration;
- a new protocol adapter or bridge executable rather than edits to the current
  bridge;
- a new dated Teensy wrapper and a new, separately named PlatformIO file rather
  than edits to the current firmware or selected build;
- offline simulation fixtures and motor-off verification tools.

Every new feature must default to disabled. Nothing in this branch may alter a
boot service, flash a Teensy, start a mission, or authorize field motion.

## Approval gates

1. Baseline containment: guard test passes and the existing software suite
   remains unchanged.
2. Offline logic: RPi supervisor state-machine tests cover expiry, recovery,
   time limit, distance limit, local-check failure, and reconnect churn.
3. Protocol simulation: dropped, delayed, duplicated, malformed, and reordered
   Pi command frames always reach Teensy Pause/neutral behavior within the
   generic command lease.
4. Diff review: separately approve the exact additive files before any build
   is copied to tractor01.
5. Motor-off test: separately approve flashing and stationary testing with the
   engine off, blades off, rear transmission lever out, physical Pause selected,
   and wheels chocked.
6. Blades-off mission: separately approve only after motor-off evidence is
   reviewed.
