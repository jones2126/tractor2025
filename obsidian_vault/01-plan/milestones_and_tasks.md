
---

> [!note] How this list is organized
> **Today** is the short working queue. **Next Field Test** is the checklist for the next outing. Sections **A–G** are the canonical backlog by subsystem. Routine start-up and shut-down checklists stay at the top for field use.
>
> Last workflow review: **2026-09-10**

## 🟡 Field Test Start-up
- [ ] Power up the RTK base, base router, and Base Starlink Mini. Turn off the router switch on the back before connecting power, then turn it on and watch its LED. Allow approximately 3 minutes for `TractorField` to appear.
- [ ] Confirm `TractorField` appears in the laptop Wi-Fi list and connect to it.
- [ ] If needed, open the Starlink app and confirm the Starlink Mini is online.
- [ ] Open the [GL.iNet router page](http://192.168.10.1/webpages/index.html#/login) and confirm the RTK base, laptop, and tractor are connected.
- [ ] Log on to the RTK base and download/reset the previous ESP32 log: `python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete`. Confirm it reports `SUCCESS: Download and delete completed`, including a non-zero line count and saved filename. Wait briefly, then run the same command with `status` in place of `download_delete` and confirm the new source log is growing.
- [ ] Run the base station survey if needed — see the runbook:
  1. `cd /home/al/tractor2025/RTKBase/setup`
  2. Run `./skytraq_rtk_base_survey.sh`. The `Run-time survey length` value should count down. If it does not, check that the antenna is connected to the port in line with the USB connector.
  3. Review the generated candidate: `python3 skytraq_rtk_base_commit_step_2.py --config "<candidate.json>"`
  4. Apply it only after review: `python3 skytraq_rtk_base_commit_step_2.py --commit --config "<candidate.json>"`
- [ ] Walk the planned route and remove new obstacles. Check the GPS antennas, wiring/connectors, tires, steering linkage, and visible fluid leaks.
- [ ] Disengage the mower deck and keep it disengaged for calibration missions. Keep people and animals clear of the route.
- [ ] Turn on the handheld radio, select **Pause**, and confirm radio connectivity. Put its transmission control in the known Manual-neutral range rather than relying on the physical center indent.
- [ ] Power on tractor01. Keep it in **Pause** and keep the e-stop immediately accessible.
- [ ] Check LED4 on the radio control for green (RTK Fix).
- [ ] Confirm router status from the GL.iNet page; if diagnostics are needed, run this on tractor01: `python3 /home/al/tractor2025/tractor_rpi/testing/router_wifi_tcp_listener_TESTING.py`
- [ ] Record the deployed revision for the run: `cd /home/al/tractor2025 && git log -1 --oneline`. Confirm it is the expected revision and do not pull or change code immediately before driving unless the change has been reviewed.
- [ ] Run the consolidated stationary check while the tractor is in Pause: `cd /home/al/tractor2025 && sudo python3 tractor_rpi/testing/mission_preflight_20260804.py`
  - Continue only when the final line is `MISSION PREFLIGHT PASS`.
  - If it fails, do not select Auto. The check already covers the mission-critical services, devices, corrections, RTK position, heading, stationary speed, steering, and JRK telemetry.
  - `tractor_rpi/check_services.sh` is optional troubleshooting; its disabled LED-controller result is not currently a navigation prerequisite.
- [ ] In **Manual**, drive to the mission's reviewed starting position. Stop, select **Pause**, and point the tractor in the expected starting direction.
- [ ] For the complete back-yard mission, use NoMachine from the on-site laptop to reach the development PC, then start the live dashboard from PowerShell on the development PC with this single line: `ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"`
  - `192.168.193.76` is tractor01's ZeroTier address. Enter the SSH-key passphrase and tractor01 `sudo` password if requested. Keep this PowerShell window open for the entire mission.
  - In the development PC's browser, open `http://192.168.193.76:8088/?key=...`, replacing `...` with the temporary operator key printed in PowerShell. If the printed URL uses tractor01's local address, retain its complete `?key=` value but replace only the host with `192.168.193.76`.
  - Keep the handheld in **Pause**, verify live telemetry, and use **START MISSION**; the dashboard launcher reruns pre-flight and the mission start-position/heading checks.
  - After dashboard **PAUSE**, first select handheld **Pause**, press **CLEAR PAUSE**, confirm the dashboard shows **HANDHELD PAUSE**, and only then select Auto. Manual-to-Auto does not clear a dashboard software Pause.
- [ ] For the optional Ring 13 1.2/1.5/1.8 m/s test, run this single-line command only after flashing its matching firmware: `cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/mission_plans/20260908_ring13_1p2_1p5_1p8_calibration && ./ring13_1p2_1p5_1p8_20260908.sh`
  - The launcher rebuilds and validates the mission, reruns pre-flight, checks the start position and heading, starts the field logger, and starts Pure Pursuit.
  - Do **not** start a separate field logger for this mission; the launcher will reject an already-running logger.
  - Type the exact requested confirmation only after every check passes. Remain in Pause until the controller is ready and its output looks normal, then select Auto only when the route is clear.
  - Stay beside the controls. For an abnormal condition, select **Pause** immediately; use the e-stop if necessary. `Ctrl+C` stops the controller and the launcher's field logger.
- [ ] For a test whose launcher does **not** manage logging, start `field_test_logger_20260828.py` separately, note its output filename and PID, confirm its row count increases, and stop it cleanly when the test ends.

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

- [ ] If the optional higher-speed test is run, deploy and flash `teensy_main_20260908_1p8_test.cpp`, restart `teensy-bridge.service`, and require pre-flight to report `Teensy firmware identity ... teensy_main_20260908_1p8_test` before running its launcher.
- [ ] Improve `configure_heading_f9p_20260727.py` handling of UBX-CFG-VALSET acknowledgements: retry or continue to verified readback when an ACK is missed, clearly distinguish ACK-ACK from ACK-NAK, and report whether each requested setting actually persisted. On 2026-09-08 the utility reported `no ACK received for UBX 06/8A`, but after the service restarted the complete mission pre-flight passed and NAV-RELPOSNED was healthy at approximately 5 Hz.
- [ ] Expand RTK logging so the next fix-loss event can be diagnosed (RELPOSNED flags, baseline length, carrier state, correction state, and per-receiver validity).
- [x] Plot cross-track error by speed for the four-value run. Median and mean error were nearly flat from 1.0 through 1.5 m/s; retain median, mean, 95th percentile, maximum, and percentage above 0.50 m for future comparisons.
- [ ] Resolve the steering asymmetry: measured minimum radius was approximately 1.63 m right versus 1.05 m left.
- [x] Verified that the historical `jrk_current` field is JRK feedback position, not amps. The optional 1.8 m/s firmware and logger add `jrk_motor_current_mA`, recent peak current, and a current-valid flag.


### Recently completed

- [x] Completed and analyzed the 2026-09-08 four-value Ring 13 run with no Teensy radio-loss mode, no JRK errors, and matching `teensy_main_20260908` identity.

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
