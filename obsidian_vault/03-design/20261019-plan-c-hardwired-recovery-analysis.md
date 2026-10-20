# Plan C — hardwired last-resort controller (assessment only)

Status: concept/hazard analysis; **not approved and not implemented**

## Intended role

A tethered controller could be a last resort when both ZeroTier and the onboard
router are unavailable. It must enter the existing Teensy safety architecture,
not drive the steering H-bridge or JRK actuator directly.

## Required interface properties

- Deliberate keyed or guarded ownership selection with a distinct HARDWIRED
  RECOVERY state; default and disconnect state is Pause.
- Heartbeat interpreted by the Teensy with a timeout no longer than the current
  500 ms watchdog. A broken/shorted tether cannot create continuous motion.
- Center-biased spring-return drive and steering demands, bounded initially to
  the same 25% reverse / 60% forward envelope.
- Separate guarded enable plus normally closed stop loop; the existing physical
  E-stop remains electrically authoritative.
- No direct bypass around Teensy mode logic, transmission neutral, freshness,
  or software ownership arbitration.
- Connector pinout designed so any single open circuit, unplug, or plausible
  adjacent-pin short produces Pause/neutral, not motion.
- Source/mode telemetry and logging through the existing bridge where possible.

## Principal hazards and mitigations

| Hazard | Required mitigation before design approval |
|---|---|
| Competing phone/mission/tether commands | One Teensy-enforced owner; break-before-make arbitration; visible mode feedback |
| Broken or unplugged cable | Pull-down/default Pause plus heartbeat timeout |
| Shorted signal or stuck control | Plausibility/range checks, spring return, enable-to-run, watchdog |
| Direct actuator bypass | Prohibit wiring to IBT-2/JRK command inputs outside Teensy ownership |
| Loss of physical E-stop authority | Keep E-stop in the final safety chain and test it independently |
| Unexpected neutral calibration | Retain neutral target handling and verify JRK feedback before motion |
| Tether snag/run-over | Strain relief, breakaway connector, routing limits, operator position rules |
| Water/EMI/ground fault | Rated connector, shielding strategy, common-ground review, protected inputs |

## Decision gates before implementation

1. Choose whether the tether connects to a dedicated Teensy input/serial port or
   a safety-rated intermediary; document failure modes for that exact circuit.
2. Produce schematic, pin-level fault table, ownership state machine, and
   electrical protection review—especially given the prior ground-difference
   Teensy damage.
3. Bench-test every single-open and adjacent-short condition with actuators
   disconnected, then repeat stationary and wheels-raised validation.
4. Obtain explicit approval before purchasing parts, changing wiring, or coding
   this Plan C path.
