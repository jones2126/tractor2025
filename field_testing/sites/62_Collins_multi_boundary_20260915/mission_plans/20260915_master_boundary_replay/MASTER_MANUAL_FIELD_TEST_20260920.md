# 62 Collins reviewed master/manual route — supervised blades-off field test

This is the field-test build of the 2026-09-20 v2 visual review. It is **not** an autonomous mowing approval. The mower deck must stay disengaged. Keep the handheld with you and a clear stop area available.

## What is in this build

- Starts at **original W1** of the 19,250-point archived resume mission, not at W12652 or the previous continuation start.
- Preserves the reviewed W12652→M18, M18–M2860, M2860→W12845 route, both garden-right shortcuts, and all three garden-left circle removals.
- Replaces only the interior of the recorded manual section with 874 distance-resampled points. Exact M18 and M2860 endpoint coordinates are retained. The original 2,843-point manual trace remains in the separate v2 review files.
- 19,825 mission waypoints, 2.00 m lookahead, and 1.00 m/s commanded at every waypoint. The manual path target spacing is 0.15 m; maximum geometric deviation from the recorded trace is 0.035 m. Maximum mission waypoint gap is 0.503 m.
- Nominal path length is approximately 3.01 km, or at least 50 minutes of motion at 1.00 m/s, plus any stops and turns.
- Garden-right center stripe coverage remains absent. This mission does not complete that uncut area.

The generated package is in `generated_master_manual_field_20260920/`. All non-resampled mission rows are copied exactly from the reviewed v2 draft. The immutable mission checksum checked by the launcher is `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`.

## Before the field

Commit and push the new launcher, dashboard wrapper, verifier, and generated package from the development repository. The tractor must pull that exact commit. Do not use an older dashboard file; it targets a different mission.

On tractor01, with the deck disengaged and handheld in Pause, run the verification first:

```bash
cd /home/al/tractor2025
git pull --ff-only origin main
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260915_master_boundary_replay/run_62_Collins_master_manual_field_20260920.sh --verify-only
```

The verification must report **PASS: exact reviewed and resampled field mission verified**. It does not start the logger or controller.

## At the field

1. Leave the deck disengaged. Put the handheld in Pause. Confirm the radio is responsive, and run the normal physical/electrical checks.
2. From the field laptop, launch only the dedicated dashboard:

   ```powershell
   ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 tractor_rpi/pure-pursuit/mission_dashboard_master_manual_field_20260920.py"
   ```

3. Open the printed dashboard URL on the phone and press **START VOICE GUIDANCE**. Voice guidance can help locate **W1** and remains enabled after arrival so it can transition directly into mission safety-stop monitoring. Before launch, the launcher requires RTK Fixed, fixed-carrier heading, baseline 0.80–1.30 m, heading accuracy at most 1°, position within 1.50 m of W1, and heading within 20°. It also runs the mission preflight, including firmware identity and neutral/safe-mode checks.
4. Review the dashboard and keep the handheld in Pause until the controller is waiting and the area is clear. Only then select Auto. Watch the first connector, the recorded manual turns, and each shortened garden join closely. If tracking or speed is wrong, immediately select handheld Pause and stop the run.
5. During the mission, the controller uses the September 18 runtime workflow: RTK Fixed plus `headValid` is the GPS/heading driving gate; carrier state, baseline, and heading-accuracy fields remain logged but do not independently stop the tractor. A GPS/heading safety loss stops motion immediately, and the controller automatically resumes with zero added stable-time delay when the basic gate recovers and guarded forward path reacquisition succeeds. No handheld mode cycle is required. The dashboard repeats its spoken safety-stop status every 10 seconds while stopped. Radio loss still stops the tractor and requires the handheld link to recover. Select Pause at any time if the automatic recovery is unsuitable or cross-track error is excessive.
6. When finished, select handheld Pause, then press Ctrl+C in the laptop's dashboard terminal. This closes the dashboard and stops the active mission safely. Confirm the tractor is stopped before leaving it.

Field logger CSVs go to `/home/al/field_logs/20260920_master_manual_field_1mps/`. The Pure Pursuit controller also writes its normal pursuit log. Preserve both, plus relevant service journals, for analysis before another run.

The prior field test lost NRF24 connectivity and experienced brief RTK/heading interruptions. The automatic GPS/heading recovery policy is intended to make those short interruptions measurable without turning each one into an operator-latched stop. If NRF24 connectivity does not recover, tracking becomes unsafe, or cross-track error is excessive, select Pause and investigate rather than attempting to finish the route. The recorded source trace contains 22 DGPS samples; the 0.15 m resampling does not improve their original positional accuracy.
