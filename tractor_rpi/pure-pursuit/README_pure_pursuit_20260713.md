# Pure Pursuit Navigation — Folder Guide (2026-07-13)

Standalone Pure Pursuit path-follower for the tractor. It reads GPS fixes,
chases a lookahead point along a loaded path, and streams `cmd_vel` (linear
speed + steering PWM) as JSON over **UDP port 6004**.

## Files

| File | Purpose |
|------|---------|
| `pure_pursuit_grok_20251112.py` | The controller. Loads a mission, computes steering + speed, sends `cmd_vel` over UDP 6004. |
| `make_mission_20260713.py` | Mission generator. Produces a densified A→B→A waypoint file with a feasible teardrop turnaround at B. |
| `mission_out_and_back_20260713.txt` | The generated mission (see coordinates below). Regenerate any time with the generator. |

## Mission file format

One waypoint per line, space-separated:

```
lat lon yaw_rad lookahead_m speed_mps
```

- `lat lon` — decimal degrees. **The first line sets the local-frame origin.**
- `yaw_rad` — heading in the controller's **math frame: CCW from EAST**
  (this is *not* a compass heading; compass→math is `radians(90 - compass_deg)`).
  Only the final waypoint's yaw affects steering; intermediate yaws are ignored.
- `lookahead_m` — pure-pursuit lookahead distance for that waypoint.
- `speed_mps` — commanded linear speed at that waypoint.

Pure pursuit selects an **actual waypoint** as its target (it does not
interpolate between them), so paths must be **densely sampled** — the generator
drops a point every 1 m.

## The current mission (`mission_out_and_back_20260713.txt`)

- **A (start / home):** 40°29'08.0"N 80°19'57.1"W
- **B (far point):** 40°29'06.9"N 80°19'56.5"W
- Leg A→B: ~36.6 m. Return B→A: ~36.6 m. Total 85 waypoints.

Because a forward-only Ackermann tractor can't reverse course in place, the
turnaround at B is a **teardrop loop** (R = 3.0 m, ~189° sweep). It bulges
~6 m to the **left** of the A–B line and ~3 m past B, so B needs open ground on
that side. The path returns to A within ~0 cm, and the tightest turn anywhere is
~3.0 m radius — inside the ~1.77 m steering limit, so every waypoint is drivable.

```
        ___
       /   \        teardrop loop, R = 3 m
A ────────── B      outbound leg, 36.6 m
   <-- return leg (rejoins the line, back to A)
```

### Regenerating / tuning the mission

Edit the constants at the top of `make_mission_20260713.py`, then re-run it:

```bash
python3 make_mission_20260713.py
```

| Constant | Meaning |
|----------|---------|
| `A`, `B` | Endpoints in decimal degrees. |
| `TURN_RADIUS_M` | Loop radius (min feasible ~1.77 m; bigger = gentler but wider). |
| `TURN_SIDE` | `"left"` or `"right"` — which way the teardrop bulges. |
| `SPACING_M` | Waypoint spacing (m). |
| `LD_STRAIGHT`, `V_STRAIGHT` | Lookahead / speed on the straight legs. |
| `LD_ARC`, `V_ARC` | Lookahead / speed on the turnaround arc (tighter/slower). |

The generator prints leg lengths, sweep, an end-vs-A error, and a feasibility
check (`R >= min radius`).

## Running the controller

**Dry run first — no tractor motion.** Watch the `cmd_vel` output on UDP 6004:

```bash
# terminal 1 — listen:
nc -u -l 6004

# terminal 2 — interactive: type "lat lon heading_deg" (heading = CCW from east):
python3 pure_pursuit_grok_20251112.py mission_out_and_back_20260713.txt --mode interactive
# example input near A heading toward B:  40.48555 -80.33252 -67
```

Modes and options:

- `--mode interactive` — you type each GPS fix; the controller computes and
  sends one `cmd_vel` per input. Good for stepping through the path by hand.
- `--mode timed` — sends at a fixed rate (default 20 Hz) using the **last pose
  provided**. It does **not** read GPS on its own (see limitation below).
- `--ip <addr>` — target IP (default `127.0.0.1`). Use the tractor's address for
  a real run.
- `--port <n>` — UDP port (default 6004).
- `--rate <hz>` — send rate for timed mode (default 20).

The output JSON looks like: `{"linear_x": <m/s>, "angular_z": <pwm 0-1023>, "timestamp": <epoch>}`
where PWM 512 = straight.

## Known limitations / before a powered run

1. **No live-GPS glue yet.** Neither mode ingests the RTK JSON off UDP 6002.
   Closing the loop on the real tractor requires code that reads 6002, extracts
   lat/lon/heading, and feeds `compute_steering` every cycle. Not built.
2. **Verify the steering sign on the bench.** `angle_to_pwm` maps a left-turn
   command to PWM < 512; confirm on jack stands that a commanded left actually
   turns the wheels left (RC pot reference: right=1, center≈503, left=1024)
   before driving under power.
3. **Heading convention** is CCW-from-east, not compass. Convert compass
   headings with `radians(90 - compass_deg)`.

## Fix history

- 2026-07-13: Corrected a projection bug in `latlon_to_xy` — the meters-per-
  degree constants (111320 / 110540) were being multiplied by *radians* instead
  of *degrees*, shrinking all distances by ~57×. Now uses degree deltas.
