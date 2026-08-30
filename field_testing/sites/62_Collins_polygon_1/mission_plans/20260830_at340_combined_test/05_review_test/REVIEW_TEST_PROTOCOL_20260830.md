# Combined 14-Ring + 21-Stripe REVIEW_TEST Protocol

Status: static validation `PASS`; first supervised field run not yet completed.

## Mission

`62_Collins_combined_14ring_21stripe_REVIEW_TEST_20260830.txt`

- 4,124 waypoints
- 2,028.5 m route
- 14 clockwise inward rings
- 12.30 m contained LRL transition from the actual spiral endpoint
- 21 east/west core stripes
- 20 contained RLR/LRL keyhole connectors
- mission speeds: 0.50 to 0.85 m/s
- first-run controller cap: 0.50 m/s
- expected first-run duration: approximately 68 minutes
- SHA-256: `386aa4913ce758a4829ef9809d34ea242cfda6deefc5a33c98316819187a3bb8`

The first run is blades off and directly supervised. It is intended to collect
tracking evidence, not mow the site.

## Development-machine checkpoint

Open and inspect:

```powershell
Invoke-Item 'C:\Repos\tractor2025\field_testing\sites\62_Collins_polygon_1\mission_plans\20260830_at340_combined_test\05_review_test\combined_review_test_preview_20260830.png'
Invoke-Item 'C:\Repos\tractor2025\field_testing\sites\62_Collins_polygon_1\mission_plans\20260830_at340_combined_test\05_review_test\combined_validation_strict_20260830.png'
```

Confirm the magenta transition begins at the innermost blue spiral endpoint,
remains in the already traversed ring area, and reaches the beginning of the
first green stripe. Orange keyholes must remain between the core and outer
rings.

After accepting the plots, commit and push the new builder, workflow, mission,
reports, plots, launcher, and this protocol. Do not include unrelated working
files merely because they are untracked.

## Tractor synchronization

On tractor01, with the tractor in Pause and blades disengaged:

```bash
cd /home/al/tractor2025
git status --short
git pull --ff-only origin main
git log -1 --oneline
```

Stop if `git status --short` reports unexpected local changes or if the pull is
not fast-forward.

Verify services:

```bash
systemctl is-active rtcm-server.service teensy-bridge.service led-controller.service
```

Expected: RTCM and Teensy bridge `active`; LED controller may remain
intentionally `inactive`.

## Mission verification

```bash
test_dir=/home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/mission_plans/20260830_at340_combined_test/05_review_test
sha256sum "$test_dir/62_Collins_combined_14ring_21stripe_REVIEW_TEST_20260830.txt"
```

Expected hash:

```text
386aa4913ce758a4829ef9809d34ea242cfda6deefc5a33c98316819187a3bb8
```

## Physical setup

1. Secure the AT340 antenna, adapter, and coax so none can move.
2. Keep the deck disengaged for the entire test.
3. Confirm the field and all keyhole areas are clear.
4. Position the tractor within 1.5 m of the mission start:
   `40.485616704, -80.332356671`.
5. Point approximately 164 degrees compass, south-southeast.
6. Keep the mode switch in Pause.
7. Keep the operator beside the E-stop with unobstructed visibility.

## Start the supervised test

The launcher reruns mission preflight, verifies the mission hash and static
reports, checks RTK Fixed, `headValid`, fixed heading carrier, start position,
and start heading, then starts the field logger and controller.

```bash
cd /home/al/tractor2025
bash field_testing/sites/62_Collins_polygon_1/mission_plans/20260830_at340_combined_test/05_review_test/run_combined_14ring_21stripe_REVIEW_TEST_20260830.sh
```

Read every displayed warning. To proceed, the launcher requires this exact
confirmation:

```text
RUN REVIEW TEST BLADES OFF
```

Remain in Pause until the controller has loaded the path and its initial output
looks correct. Select Auto only when ready to begin. Use Pause immediately for
unexpected steering, heading, speed, route position, carrier loss, obstacles,
or inadequate clearance. `Ctrl+C` stops the controller and logger.

## Observation priorities

Record or mark the approximate time of:

1. mission start;
2. each corner where tracking error appears significant;
3. the final two spiral rings;
4. entry into the magenta spiral-to-stripe transition;
5. entry onto stripe 1;
6. the first keyhole on each side;
7. any heading carrier `none` or invalid-heading stop; and
8. mission completion or operator abort.

The most important new evidence is whether the tractor follows the 12.30 m LRL
transition smoothly and arrives aligned with stripe 1, followed by whether the
1.90 m nominal keyholes are repeatable at the 0.50 m/s controller cap.

## After the run

The launcher prints the field-log location during cleanup. Copy that CSV to a
new dated run folder on the development machine. Preserve the Pure Pursuit
per-cycle log as well. Compare:

- commanded path versus RTK position;
- cross-track and heading error by spiral, transition, stripe, and keyhole;
- carrier state and `headValid` interruptions;
- speed and steering saturation through turns;
- actual minimum turn radius; and
- every Pause or abort event.

Do not increase the speed cap or engage the mower until the logged transition
and keyhole results have been reviewed.
