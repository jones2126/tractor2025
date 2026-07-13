# Pure Pursuit First-Mission Test Protocol (2026-07-13)

Controller: `pure_pursuit_live_20260713.py` · Mission: `mission_out_and_back_20260713.txt`
cmd_vel contract: `{"linear_x": -1..0 (neg=fwd), "angular_z": -1..+1 (+1=full left), "timestamp": epoch}` on UDP 6004.

**Prerequisite:** Teensy firmware auto-mode (mode 2) steering handler must be updated to map
normalized angular_z (-1..+1) to pot counts (197/447/815). Do NOT run against old firmware
that expects raw pot counts — a 0.07 command would be read as pot count 0 (hard right slam).

---

## Phase 0 — Desk / no hardware (RPiNAS or tractor02, any time)

Goal: confirm the controller's math and message format with no tractor involved.

- [ ] `nc -lu 6004` in one terminal.
- [ ] Interactive dry run in another:
  ```bash
  python3 pure_pursuit_live_20260713.py mission_out_and_back_20260713.txt --mode interactive
  ```
  Enter `40.48555 -80.33252 -67` (near A, heading toward B, math-frame degrees).
- [ ] PASS criteria:
  - `Loaded 85 waypoints` prints.
  - nc shows JSON with `linear_x` ≈ **-0.5** (negative!) and `angular_z` between -1 and +1.
  - Walking a few fixes along the A→B line keeps `steer` near 0; a fix offset left of
    the line produces negative steer (right correction), offset right produces positive.
- [ ] Replay test (optional, tractor02): with rtcm-server running on the bench,
  `--mode live --min-fix any --allow-head-invalid` and watch it consume real 6002 packets.
  The tractor isn't attached, so nothing moves — you're validating the GPS listener,
  compass→math heading conversion, and staleness gate. Kill rtcm-server mid-run and
  confirm `[WAIT] not driving: stale or no GPS` appears within ~0.5 s.

## Phase 1 — Firmware update + jack stands (tractor01, engine on, wheels off ground)

Goal: verify the new normalized-steering firmware and the steering SIGN through the full
UDP path. Manual field driving so far used the RC radio path (mode 1); this exercises the
untested UDP 6004 → bridge → `CMD,x,z` → mode 2 path.

Setup: tractor on jack stands, e-stop within reach, RC mode switch to AUTO (mode 2),
rtcm-server + teensy-bridge services running.

- [ ] Flash updated Teensy firmware (normalized steering map in mode 2 case).
- [ ] Manual single commands first (no pure pursuit yet):
  ```bash
  python3 - <<'EOF'
  import socket, json, time
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  def cmd(lx, az):
      s.sendto(json.dumps({"linear_x": lx, "angular_z": az, "timestamp": time.time()}).encode(), ("127.0.0.1", 6004))
  cmd(0.0, 0.0); time.sleep(2)      # center — pot should settle ~447
  cmd(0.0, 0.5); time.sleep(3)      # half LEFT — wheels must turn LEFT, pot toward 631
  cmd(0.0, 0.0); time.sleep(2)
  cmd(0.0, -0.5); time.sleep(3)     # half RIGHT — pot toward 322
  cmd(0.0, 0.0)
  EOF
  ```
- [ ] PASS criteria (watch wheels + UDP 6003 steering setpoint/pot values):
  - `+0.5` → wheels visibly LEFT, setpoint ≈ 631.
  - `-0.5` → wheels visibly RIGHT, setpoint ≈ 322.
  - `0.0` → returns to center, pot ≈ 447.
  - Stop sending for >1 s → Teensy's cmd_vel timeout holds position (no runaway).
- [ ] **If +0.5 turns the wheels RIGHT: stop.** Fix the sign in the firmware map (or flip
  `angle_to_normalized`'s sign), never both. Re-test before proceeding.
- [ ] Transmission sanity (still on stands): `cmd(-0.3, 0.0)` → wheels rotate FORWARD.
  `cmd(0.0, 0.0)` → neutral. If -0.3 gives reverse, the linear_x sign convention in the
  firmware/bridge differs from the documented one — resolve before Phase 2.

## Phase 2 — Speed calibration (tractor01, on the ground, open flat area)

Goal: replace the `--max-speed 1.0` placeholder with a measured number.

- [ ] Mark a 10 m course (tape measure).
- [ ] Using the manual sender from Phase 1, command a fixed `linear_x` (start at -0.3),
  time the 10 m traverse, compute m/s. Repeat for -0.5.
- [ ] Compute the effective full-scale speed: if -0.5 gives 0.42 m/s, then
  max_speed ≈ 0.84 m/s → run pure pursuit with `--max-speed 0.84`.
  (Note: the transmission bucket mapping is stepped, not linear — two data points tell
  you if linearity is close enough for mission speeds of 0.3–0.5 m/s. If badly nonlinear,
  we add a lookup table later; for the first mission a conservative single scale is fine.)
- [ ] Record the numbers in the Obsidian vault (04-reference).

## Phase 3 — Live-mode gate check (tractor01, on stands OR ground with e-stop armed)

Goal: prove every safety gate fires before trusting them in the field.

```bash
python3 pure_pursuit_live_20260713.py mission_out_and_back_20260713.txt --mode live --max-speed <measured>
```

- [ ] Base station up ≥5 min (needs TIME mode + Type 1005 before rover converges).
- [ ] With RTK Fixed: status line prints `fix=RTK Fixed`, commands flow.
- [ ] Cover/disconnect the base antenna path (or stop the base): within seconds of the
  rover dropping to Float, `[WAIT] not driving: fix_quality='RTK Float' ...` prints and
  cmd_vel goes to stop. Wheels center, transmission neutral.
- [ ] `sudo systemctl stop rtcm-server` → `[WAIT] ... stale or no GPS` within 0.5 s. Restart it.
- [ ] Ctrl+C the controller → FINAL STATISTICS prints AND one last stop command is sent
  (verify on 6003 that setpoint recentred / bridge saw the stop).
- [ ] Known bug reminder: `head_valid` instability is an open item. If `[WAIT] headValid=False`
  chatters during this phase, that's the known bug surfacing — note the frequency. Do not
  work around it with `--allow-head-invalid` for a powered ground run.

## Phase 4 — First mission (tractor01, field, mission area clear)

Preconditions: Phases 0–3 all PASS; teardrop area at B has ≥6 m clear to the LEFT of the
A–B line and ~3 m beyond B; person walking alongside with the RC transmitter — flipping
the mode switch out of AUTO is your software e-stop, the physical e-stop is the hard stop.

- [ ] Start `field_test_logger` first — you want the CSV of this run regardless of outcome.
- [ ] Position tractor at A, pointed roughly at B (within ~±20°).
- [ ] Start controller:
  ```bash
  python3 pure_pursuit_live_20260713.py mission_out_and_back_20260713.txt --mode live --max-speed <measured>
  ```
- [ ] Watch the console: `idx` should advance steadily; `delta` small on the straight,
  larger (toward the steering limit) in the teardrop; `fix=RTK Fixed` throughout.
- [ ] Abort criteria — flip out of AUTO immediately if:
  - cross-track error visibly growing (weaving amplitude increasing rather than damping),
  - steering saturated (steer=±1.00) on the straight leg,
  - `idx` stuck while the tractor keeps moving,
  - anything unexpected at all. Cheap to abort; the log tells the story.
- [ ] Success criteria: completes A→B, teardrop, B→A; "Goal reached -- sending stop"
  prints; stops within ~pos_tol of A.
- [ ] Post-run: pull the field_test CSV, run the Folium analysis, overlay actual track
  vs. mission waypoints. Tuning knobs if tracking is loose: lookahead values in the
  mission file (bigger = smoother/lazier, smaller = tighter/twitchier), speed on the arc.

## Quick reference

| Symptom | Likely cause |
|---|---|
| `[WAIT] stale or no GPS` forever | rtcm-server not running, or port conflict — check `ss -ulpn | grep 6002` |
| Wheels slam to a lock at start | Firmware still expects pot counts in angular_z — Phase 1 prerequisite skipped |
| Tractor creeps backward | linear_x sign flipped somewhere — Phase 1 transmission check skipped |
| Weaves symmetrically about the line | Lookahead too short for speed — increase ld in mission file |
| Consistently cuts one side of the teardrop | Asymmetric lock angles (368 vs 250 counts) — note it, revisit later |
| `headValid=False` chatter | Known open bug — log it, don't bypass |
