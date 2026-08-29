
---

> [!note] How this list is organized
> **Today** is the short working queue. **Next Field Test** is the checklist for the next outing. Sections **A–G** are the canonical backlog by subsystem. Routine start-up and shut-down checklists stay at the top for field use.
>
> Last workflow review: **2026-07-27**

## 🟡 Field Test Start-up
- [ ] Power up RTKBase, Base Router, Base Starlink Mini - Suggestion: Turn off router switch on the back; Connect; Then turn on and watch LED to confirm power.  Takes 3 minutes for network TractorField to appear.
- [ ] Check if TractorField is being broadcast (e.g. Check if TractorField is showing in laptop wi-fi list)
- [ ] If needed, open Starlink App; Check if Starlink Mini is online
- [ ] Once you have TractorField you can log on to the router and see what is connected http://192.168.10.1/webpages/index.html#/login  ; watch to make sure Base, Laptop and tractor are connected
- [ ] Log on to RTK Base and reset download files $ python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete ; Then check status and make sure file is growing
- [ ] Run the base station survey if needed — see the runbook:
  1. `cd /home/al/tractor2025/RTKBase/setup`
  2. Run `./skytraq_rtk_base_survey.sh`. The `Run-time survey length` value should count down. If it does not, check that the antenna is connected to the port in line with the USB connector.
  3. Review the generated candidate: `python3 skytraq_rtk_base_commit_step_2.py --config "<candidate.json>"`
  4. Apply it only after review: `python3 skytraq_rtk_base_commit_step_2.py --commit --config "<candidate.json>"`
- [ ] Turn on radio - check radio connectivity
- [ ] Power on tractor01
- [ ] Log on to the GL.iNet router and check that the status script is running; or, from tractor01, run `python3 /home/al/tractor2025/tractor_rpi/testing/router_wifi_tcp_listener_TESTING.py`
- [ ] Run sudo python3 /home/al/mission_preflight_20260804.py  (update location to tractor2025\tractor_rpi\testing - this is a consolidated safety check)
- [ ] Check LED4 on radio control for green (i.e. RTK Fix)
- [ ] Check key services are running: $ cd /home/al/tractor2025/tractor_rpi  ; $ ./check_services.sh
- [ ] Start data logging -- Use nohup with output redirected to a log file: $ nohup python3 /home/al/tractor2025/tractor_rpi/field_test_logger_20260828.py > /home/al/field_logs/logger_console.log 2>&1 &
- [ ] Note the PID it prints (e.g. [1] 12345) so you can kill it cleanly later: $ kill 12345

---

## 🟡 Field Test Shut-Down

- [ ] Stop the mission normally and allow both loggers to close cleanly.
- [ ] Record the **site name** and **run ID** shown in the log filenames. The run ID has the format `YYYYMMDD_HHMMSS`, for example `20260724_172543`.
- [ ] On the Windows analysis computer, open PowerShell at the repository root:
  ```powershell
  Set-Location C:\Repos\tractor2025
  .\field_testing\tools\field_test_analysis_menu_20260726.ps1
  ```
- [ ] Select the correct site and run ID. Choose **N** to enter a new run ID if it has not been downloaded before.
- [ ] Choose menu option **2 — Download tractor logs and mission package** while tractor01 is still powered on.
- [ ] Confirm the tractor collection reports:
  - `Tractor run collection complete`
  - Non-zero Pure Pursuit and field-logger row counts
  - A hash manifest and collection summary in `%USERPROFILE%\Documents\field_plans\<site>\runs\<run-id>\`
- [ ] Choose menu option **1 — Download RTK-base / ESP32 data** while the RTK base is still powered on.
  - Confirm the prompt that saves the ESP32 data on the RTK base and then resets the ESP32 source log.
  - Keep the verified RTK-base recovery copy when prompted unless there is a specific reason to remove it.
- [ ] Confirm the RTK-base collection reports:
  - `RTK base ESP32 collection complete`
  - A non-zero ESP32 row count
  - `Base recovery copy : True`
- [ ] Optional: choose menu option **3** to analyze the run and open the generated HTML map.
- [ ] Shut down tractor01 only after its files have been verified: `ssh al@192.168.193.76 "sudo shutdown now"`
- [ ] Shut down the RTK base only after its files have been verified: `ssh al@192.168.193.88 "sudo shutdown now"`
- [ ] Turn off radio control.
- [ ] Turn off master power to the tractor, disconnect the battery charger, put the tractor away, and cover it.



---

## 🔴 Today Action Items

- [ ] Expand RTK logging so the next fix-loss event can be diagnosed (RELPOSNED flags, baseline length, carrier state, correction state, and per-receiver validity).
- [ ] Plot cross-track error and choose a repeatable score (for example RMS and 95th percentile) for comparing runs.
- [ ] Resolve the steering asymmetry: measured minimum radius was approximately 1.63 m right versus 1.05 m left.
- [ ] Verify the `jrk_current` field name/source; it stayed at `2985`, which looks like JRK position rather than current.


### Recently completed

- [x] Verified the base-link GPS udev mapping.
- [x] Set the Pure Pursuit base-link GPS offsets to zero.
- [x] Compiled and uploaded the 2026-07-14 Teensy firmware.
- [x] Installed tractor-engine throttle control.
- [X] Check 5 V rail stability: `watch -n 1 "vcgencmd pmic_read_adc | grep -i 5v"`
- [x] Confirm ground speed is present in the GPS stream: `sudo cat /dev/gps-base-link | grep -E "RMC" | head -5`
- [x] Added battery power for the base Starlink/router and mounted them on a hand cart.
- [X] Confirm the mission runner starts both the field logger and pursuit logger, then verify that latitude, longitude, actual speed, and JRK values are changing.
- [X] Analyze the 2026-07-24 Collins Drive run and save its tracking summary with the run files.
- [X] Consolidate the field-test and pure-pursuit analysis tools into one repeatable, documented workflow.
- [X] Move the current Teensy firmware to tractor01, compile, upload, and verify the running version.
- [X] Document the final `SPEED_CAL`, manual/autonomous turn radii, RTK-drop observations, and successful mission filenames.


---

## 🟡 Next Field Test Tasks

- [ ] Get yard flags at Home Depot to mark position to drive.
- [ ] Clean the 10-turn potentiometer; Adjust the radio-control speed control, neutral is slightly counter-clockwise from its midpoint.
- [ ] Test the new PowerShell collection scripts: `collect_site_run_20260724.ps1` and `collect_rtkbase_esp32_20260724.ps1`.
- [ ] Log tractor01 CPU utilization during the run.
- [ ] Connect the wheel-odometry units, capture their data, and compare it with RTK-fix losses.
- [ ] Run a box/perimeter mission and build the next coverage plan from the captured boundary.
- [ ] Make bracket for cutting deck
- [ ] Build an antennae holder and dig a hole in the ground for it.
- [ ] Run a manual mission around plot 1 and convert it to a mission
- [ ] Run a manual mission around plot 2 and 3 in the side yard


---

## A — RPi - Main Server
*Status: IN PROGRESS*

- [ ] **X20D RTCM server**: Update `rtcm_server_x20d_{date}.py` to incorporate changes from `rtcm_server_{date}.py`.
- [ ] **tractor01 services:** copy /etc/systemd/system/rtcm-server.service and  /etc/systemd/system/teensy-bridge.service to /tractor2025/tractor_rpi/setup/.
- [ ] **tractor02 services**: Run `install_services.sh` (has not been run yet on tractor02). Then copy and enable services:
  ```bash
  sudo cp ~/tractor2025/tractor_rpi/setup/rtcm-server.service /etc/systemd/system/
  sudo cp ~/tractor2025/tractor_rpi/setup/teensy-bridge.service /etc/systemd/system/
  sudo cp ~/tractor2025/tractor_rpi/setup/led-controller.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable rtcm-server teensy-bridge led-controller
  sudo systemctl start rtcm-server teensy-bridge led-controller
  ```
- [ ] Double check `.service` files on tractor01 and tractor02 are consistent and also captured in the setup folder
- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky) - adjust after RTK Base is set
- [X] Transmission neutral confirmed via JRK position reader
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [X] Manual drive test with tractor moving
- [X] Confirm field logger is capturing GPS data with base station present.
- [X] Transmission bucket system (10 buckets, neutral=2985) - adjust after throttle is set

---

## B — Web Teleoperation
*Status: NOT STARTED*

- [ ] 3D-print holder for Oak Camera
- [ ] Browser controls working
- [ ] OAK camera feed live
- [ ] Review this summary chat looking at the test code: https://chatgpt.com/share/6a5907b1-b708-83ea-8c54-983f3b9b04ec

---

## C — GPS / RTK
*Status: IN PROGRESS*

- [ ] **RTK base station permanent setup**: Weatherproof and securely remount the electronics and antenna.
- [ ] Survey the final fixed base location for 24 hours and commit the reviewed coordinate.
- [ ] Apply latest ArduSimple firmware for X20D to achieve 10 Hz
- [ ] Add 12 volt monitoring from ESP32 directly instead of through the solar charge converter
- [ ] Update `ntfy.sh` boot notification script to also publish: - `hostname -I` (all IP addresses) especially WLAN and ETH addresses; Confirm ZeroTier is up and reachable
- [ ] Add weather proof 12V rocker switches for main components
- [X] Test achieving RTK Fix and heading live
- [X] Decouple RTCM forwarding failure from GPS parsing — base station absence should not block GPS data (field test confirmed this bug)
- [X] RTK base station RPi power issue — install new 12V→5V converter
- [x] **RTK base station mobility**: Set up battery power for Starlink/router and add the hand cart.
- [X] RTCM corrections streaming for simulated testing (e.g. non-live testing when actual base station is not available) - determined this is not possible

---

## R — Radio Manual Control
*Status: IN PROGRESS*

- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky)
- [ ] Investigate steering asymmetry and verify that the right limit really reaches hard-right; measured radii were approximately 1.63 m right and 1.05 m left.
- [ ] Add a magnetic steering-angle sensor as a more robust alternative to the potentiometer.
- [ ] Add emergency on-tractor controls for left, right, forward, and reverse so the tractor can be recovered if the radio/potentiometer fails.
- [ ] Put the status of the pushbuttons in the field logger data.  Think of a time when you want to capture a mission by first driving the path.  Having a 'bread crumb' of sorts at the beginning and end of that mission will be helpful.
- [x] NRF24 radio communication working (addresses "1Node"/"2Node")
- [x] Steering direction fixed (RPWM/LPWM swap corrected 2026-05-18)
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [x] Transmission bucket system working (10 buckets, neutral=2985)
- [X] Transmission neutral confirmed via JRK position reader
- [x] Manual drive test with tractor moving

---


## D — Pure Pursuit Navigation
*Status: IN PROGRESS*

- [ ] Test the Collins Drive coverage mission end-to-end.
- [ ] Add a safe transition-path generator that connects the tractor's current pose to the start of a planned mission (evaluate a Dubins-path approach).
- [ ] Investigate and correct the target-table error observed on 2026-07-22.
- [ ] See Claude chat (check LLM - may not be able to use Fable ): https://claude.ai/chat/d9024a5b-7510-4f24-b937-55456ee38de8
- [ ] Define acceptable cross-track-error targets, then tune toward them using the same scoring method for every run.

---

## E — Electronics Board - Gen 2 (Permanent Mount)
*Status: IN PROGRESS*

### Layout & CAD
- [ ] Physical wiring dry-run — verify component positions are reachable given real cable bend radii
- [ ] Make a custom PCB to hold Teensy 4.1 and NRF24 radio
- [ ] Laser cut enclosures to make electronics weather resistant

### Power Distribution
- [ ] Make PCB for RPi 20 pin header for Andon Light and power input.
- [ ] Put battery monitor/voltage divider on Teensy and publish tractor battery voltage data on a UDP port for the field logger.
- [x] Place and secure Buck converter #1 and #2 (RPi power) — route USB-C output to RPi
- [X] Run 14 AWG from blade fuse block to IBT-2; install 20 A fuse

### Component Placement
- [ ] Install and wire JRK G2 motor controller
- [ ] Build and install IBT-2 PCB 
- [ ] Install LED andon signal light
- [X] Install and wire relay board — confirm it doesn't block USB or Ethernet access to RPi

### Pending Fabrications
- [ ] Print bracket for JRK G2 controller
- [X] Print retention strap for Buck converters (reuse previously designed Fusion 360 model)

### Laser Cut Board
- [ ] Create Inkscape layout for electronics
- [ ] Obtain cut-file software used at Protohaven (Rabbit laser) — set up at home
- [ ] Cut board at Protohaven


---

## F — Data Analysis and Field Testing
*Status: IN PROGRESS*

- [ ] **Field logger — RTK diagnostics**: Add RELPOSNED flags, baseline length, carrier-solution state, correction status, and validity for each receiver.
- [ ] **Field logger — system diagnostics**: Add tractor01 CPU utilization and wheel-odometry data.
- [ ] **Field logger — controls**: Resolve the `jrk_current` value/source and add radio pushbutton states as breadcrumb markers.
- [ ] **RTK analysis**: Calculate fix-loss count, average duration, maximum duration, and total time without RTK Fixed.
- [ ] **Pure-pursuit analysis**: Plot cross-track error and report comparable RMS, median, 95th-percentile, and maximum values.
- [ ] Move the analysis scripts out of `Downloads` into the repository and document one command that analyzes a complete run directory.
- [ ] Automate collection and archiving of the field log, pursuit log, mission files, RTK-base log, and analysis output under a run-ID directory.
- [ ] Analyze and archive the 2026-07-24 run (`20260724_172543`) as the first example of the consolidated workflow.
- [ ] Install PostgreSQL + TimescaleDB on RPiNAS (via apt or Docker)
  - Create hypertable with columns: `timestamp`, GPS lat/lon/alt/speed, sensor readings, device ID
  - Python listener on RPi that batches inserts from tractor UDP streams
  - Log video file paths and timestamps to DB
  - Add daily compression/backup cron jobs
- [ ] Install Grafana on RPiNAS to visualize field test CSV logs and DB data
- [ ] Migrate RPiNAS OS to Ubuntu 26.04 LTS

---

## G — Maintenance, Documentation, and Project Updates
*Status: IN PROGRESS*

- [ ] Service the mower: obtain the tune-up kit, spark plug, oil filter, oil, blades, grease gun, drain pan, and shop rags.
- [ ] Prepare the next project video update.
- [ ] Write a dated field-test summary covering final `SPEED_CAL`, manual/autonomous turn radii, RTK-drop observations, and successful mission filenames.
- [ ] Sync confirmed code and documentation changes to GitHub after each field-test session.

---

## Reference - Recent LLM conversations
*Status: NA*

- **Mission-file speed setting**: https://claude.ai/chat/b728f2b7-ffa6-49cf-917c-8d0f23bbee50
- **Choosing an LLM**: https://claude.ai/chat/cd6167d1-b607-4d12-a8b2-d62f5a3703a7
- **Analysis of lost RTK Fix**: https://claude.ai/chat/475bcda0-390f-4d66-a788-43d0b955dbf9
- **Why the shell runner was linked to missing latitude/longitude in the field log**: https://claude.ai/chat/e0e147a3-70e2-4d9a-b76a-96ee17687003
