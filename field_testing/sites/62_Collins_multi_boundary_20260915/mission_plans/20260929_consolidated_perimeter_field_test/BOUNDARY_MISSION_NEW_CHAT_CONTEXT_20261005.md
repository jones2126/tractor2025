# New-chat context — run the approved consolidated boundary mission

## Paste-ready prompt

Paste the following into a new chat started in the `tractor2025` repository:

> I am at Tractor01 and want to run the approved consolidated perimeter field
> test. Read
> `obsidian_vault/02-testing/20261005-wifi-control-boundary-mission-handoff.md`,
> this context file, `field_testing/FIELD_COMMANDS_20260930.md`, and the
> package `FIELD_TEST_README.md` before guiding me. Walk me through it one step
> at a time and wait for my output after each consequential command. Use the
> existing approved package; do not rebuild or edit the mission. This is a
> directly supervised, blades-off run at 1.00 m/s. I will keep one Wi-Fi phone
> control page in the foreground and the physical E-stop available. Voice can
> remain off. Require package verification, Heading-F9P verification, and a
> complete `MISSION PREFLIGHT PASS` before Auto. Help me preserve and review
> the field log when the run ends or is stopped.

## Scope and authority

This file is an operational handoff, not a new approval. The only approved
route is the checked-in 2026-09-29 consolidated perimeter package beside this
file. It is approved solely for an initial, directly supervised, blades-off
field test.

Do not:

- regenerate, smooth, reorder, or edit the approved mission;
- bypass a verifier, preflight, dashboard, or launcher gate;
- select Auto after a warning that says not to;
- treat this run as approval for mowing or unattended operation;
- open multiple Wi-Fi control tabs;
- require voice guidance or voice notes.

## Known current state at handoff

- Tractor01 pulled Git commit `c8b24e7` on 2026-10-05.
- Firmware observed: `teensy_main_20261003_wifi_v3`.
- `tractor-wifi-control.service`, `teensy-bridge.service`, and
  `rtcm-server.service` were active during the latest checks.
- The phone-control URL is persistent across restarts.
- The visible ZeroTier page automatically reconnected in Pause in about 0.5
  seconds during the acceptance test.
- Phone Manual steering and transmission, phone E-stop latch/reset, and
  stationary Auto selection were already tested. Repeat only the checks needed
  by the current preflight and field conditions.
- The last full preflight passed, but it is not a substitute for a fresh field
  preflight.

## Immutable package facts

- Mission: `generated/62_Collins_consolidated_perimeter_1mps_20260929.txt`
- Audit: `generated/62_Collins_consolidated_perimeter_1mps_20260929_audit.csv`
- Verifier: `verify_consolidated_perimeter_20260929.py`
- Launcher: `run_62_Collins_consolidated_perimeter_20260929.sh`
- Dashboard: `mission_dashboard_consolidated_perimeter_20260929.py`
- Waypoints: 3,989.
- Route length: 790.077 m.
- Speed commands: 1.00 m/s throughout.
- Mission SHA-256:
  `35ea19776283ef415518fd57997e418b966ea823741bcc6a6ca135cee7b504d9`.
- Audit SHA-256:
  `fe2d65daaecd80b4dce1582a4e74ab4744c0fe7cb1cf9e72d6944f846353e50e`.

The generated validation report retains historical NRF-handheld and old
firmware language from its approval provenance. The current launcher,
dashboard, preflight, and firmware use the Wi-Fi-primary phone controller. Do
not change the generated report during field operations.

## Required operating sequence

The next chat should guide this sequence and examine the actual output before
advancing.

### 1. Establish safe control and network

- Tractor stationary and clear of people.
- Blades disengaged.
- Physical E-stop immediately available.
- Open exactly one current Wi-Fi phone control page and confirm Pause.
- Keep that page in the phone foreground.
- Use the mission dashboard on the Windows laptop, not in another phone tab.

### 2. Pull and verify without starting motion

Use the command in section 3 of `field_testing/FIELD_COMMANDS_20260930.md`.
Require:

`PASS: exact approved supervised blades-off field package verified.`

The exact verifier can also be run directly:

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/verify_consolidated_perimeter_20260929.py
```

### 3. Verify the Heading F9P profile

Use section 4 of `field_testing/FIELD_COMMANDS_20260930.md`. It stops
`rtcm-server`, applies the RAM-only 5 Hz moving-base profile, independently
reads back the targeted settings, and restarts the service. Require both the
configuration PASS and active service. Do not select Auto if either fails.

### 4. Run the complete fresh preflight

Phone remains connected in Pause:

```bash
cd /home/al/tractor2025
sudo python3 tractor_rpi/testing/mission_preflight_20261002.py \
  --expected-firmware teensy_main_20261003_wifi_v3
```

Require the final line:

`MISSION PREFLIGHT PASS`

Heading satellites used must be above 25 for a normal PASS. A warning at
24-25 deserves review; 23 or fewer fails closed.

### 5. Start the laptop dashboard

In a second PuTTY window:

```bash
cd /home/al/tractor2025
sudo -v
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/mission_dashboard_consolidated_perimeter_20260929.py
```

Open its printed `http://192.168.193.76:8088/?key=...` URL on the Windows
laptop. Voice guidance is optional and can stay off.

### 6. Reach and align with the start

- Use guarded phone Manual while watching dashboard distance and direction.
- Stop in the dashboard's accepted start envelope: within 1.50 m and within
  20 degrees, with RTK Fixed and fixed-carrier heading.
- Return the phone to Pause before pressing `START MISSION`.

### 7. Start and supervise

- Press `START MISSION` on the laptop dashboard only while stationary in
  phone Pause.
- Let the launcher's repeated gates complete.
- Select guarded phone Auto only after the controller is live, the dashboard
  is healthy, and the route is visibly clear.
- Supervise continuously with blades off and physical E-stop ready.
- Pause immediately for unexpected clearance, tracking, heading, control,
  network, or mechanical behavior.

### 8. Finish or stop safely

- Return phone control to Pause.
- Confirm transmission neutral and steering output stopped.
- Preserve the field log path printed by the launcher. Logs normally go under
  `/home/al/field_logs/20260929_consolidated_perimeter_1mps/`.
- Preserve dashboard/controller terminal output and any operator observations.
- Do not delete or overwrite logs after an aborted run.

## Information to bring back after the run

- Completed versus stopped/aborted.
- Field log filename.
- Final waypoint or route distance if incomplete.
- Any dashboard safety events or controller pauses.
- Any sections with poor clearance, oscillation, cross-track error, unexpected
  turns, wheel slip, or loss of RTK/heading.
- Whether the physical and phone E-stops remained immediately usable.
- Any operator notes, even if voice notes were disabled.
