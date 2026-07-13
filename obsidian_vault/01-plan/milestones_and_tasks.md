
---

## 🟡 Field Test Start-up
- [ ] Power up RTKBase, Base Router, Base Starlink Mini
- [ ] Check if TractorField is being broadcast (e.g. Check if TractorField is showing in laptop wi-fi list)
- [ ] Once you have TractorField you can log on to the router and see what is connected http://192.168.10.1/webpages/index.html#/login  ; watch to make sure Base, Laptop and tractor are connected
- [ ] Log on to RTK Base and reset download files $ python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete ; Then check status and make sure file is growing
- [ ] Run the base station survey if needed - see the runbook
- [ ] Turn on radio - check radio connectivity
- [ ] Power on tractor01
- [ ] Log on to GLINET router and check script is running and/or run $ python3 /home/al/tractor2025/tractor_rpi/testing/router_wifi_tcp_listener_TESTING.py
- [ ] Check LED4 on radio control for green (i.e. RTK Fix)
- [ ] Check key services are running: $ cd /home/al/tractor2025/tractor_rpi  ; $ ./check_services.sh
- [ ] Start data logging -- Use nohup with output redirected to a log file: $ nohup python3 /home/al/tractor2025/tractor_rpi/field_test_logger_20260710.py > /home/al/field_logs/logger_console.log 2>&1 & 
- [ ] Note the PID it prints (e.g. [1] 12345) so you can kill it cleanly later: $ kill 12345

---

## 🟡 Field Test Shut-Down

- [ ] Move field logger data to laptop from tractor01
- [ ] tractor01 - sudo shutdown now
- [ ] RtkBase - $ python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete
- [ ] RtkBase - $ sudo shutdown now



---

## 🔴 Today Action Items

- [ ] **RTK base station**: Setup battery power for Starlink and Router; Re-mount electronics; Add hand cart to make it easy to move into place; Survey location for 24 hours.
- [ ] Install a throttle control on the tractor motor


---

## 🟡 Next Field Test Tasks

- [ ] Check voltage using watch command: $ watch -n 1 "vcgencmd pmic_read_adc | grep -i 5v"
- [ ] Review the end of this chat.  https://claude.ai/chat/3c1d37b8-b622-45c0-a808-2516b6f1fcc4 ; Resolve getting ground speed into the captured data. ; $ sudo cat /dev/gps-base-link | grep -E "RMC" | head -5  # will see if RMC NMEA sentences are being published.

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
- [ ] Transmission bucket system (10 buckets, neutral=2985) - adjust after throttle is set
- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky) - adjust after RTK Base is set
- [ ] Transmission neutral confirmed via JRK position reader
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [X] Manual drive test with tractor moving
- [X] Confirm field logger is capturing GPS data with base station present.

---

## B — Web Teleoperation
*Status: NOT STARTED*

- [ ] 3D-print holder for Oak Camera
- [ ] Browser controls working
- [ ] OAK camera feed live
- [ ] Low-latency steering response

---

## C — GPS / RTK
*Status: IN PROGRESS*

- [ ] **RTK base station**: Setup battery power for Starlink and Router; Re-mount electronics; Add hand cart to make it easy to move into place; Survey location for 24 hours.
- [ ] Apply latest ArduSimple firmware for X20D to achieve 10 Hz
- [ ] RTCM corrections streaming for simulated testing (e.g. non-live testing when actual base station is not available)
- [ ] Add 12 volt monitoring from ESP32 directly instead of through the solar charge converter
- [ ] Update `ntfy.sh` boot notification script to also publish: - `hostname -I` (all IP addresses) especially WLAN and ETH addresses; Confirm ZeroTier is up and reachable
- [ ] Add weather proof 12V rocker switches for main components
- [X] Test achieving RTK Fix and heading live
- [X] Decouple RTCM forwarding failure from GPS parsing — base station absence should not block GPS data (field test confirmed this bug)
- [X] RTK base station RPi power issue — install new 12V→5V converter

---

## ? — Radio Manual Control
*Status: IN PROGRESS*

- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky)
- [ ] Transmission neutral confirmed via JRK position reader
- [ ] Manual drive test with tractor moving
- [ ] Put the status of the pushbuttons in the field logger data.  Think of a time when you want to capture a mission by first driving the path.  Having a 'bread crumb' of sorts at the beginning and end of that mission will be helpful.
- [x] NRF24 radio communication working (addresses "1Node"/"2Node")
- [x] Steering direction fixed (RPWM/LPWM swap corrected 2026-05-18)
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [x] Transmission bucket system working (10 buckets, neutral=2985)

---


## D — Pure Pursuit Navigation
*Status: NOT STARTED*

- [ ] Test first waypoint mission
- [ ] Achieve acceptable cross-track error

---

## E — Electronics Board - Gen 2 (Permanent Mount)
*Status: IN PROGRESS*

### Layout & CAD
- [ ] Physical wiring dry-run — verify component positions are reachable given real cable bend radii
- [ ] Make a custom PCB to hold Teensy 4.1 and NRF24 radio
- [ ] Laser cut enclosures to make electronics weather resistant

### Power Distribution
- [ ] Make PCB for RPi 20 pin header for Andon Light and power input.
- [ ] Put battery monitor/voltage divdier on teensy and publish tractor battery voltage data on a UDP port for field logger.
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
*Status: NOT STARTED*

- [ ]  **Field logger**: Update `field_test_logger_{date}.py` to include data needed to debug loss of RTK Fix
- [ ] Install PostgreSQL + TimescaleDB on RPiNAS (via apt or Docker)
  - Create hypertable with columns: `timestamp`, GPS lat/lon/alt/speed, sensor readings, device ID
  - Python listener on RPi that batches inserts from tractor UDP streams
  - Log video file paths and timestamps to DB
  - Add daily compression/backup cron jobs
- [ ] Install Grafana on RPiNAS to visualize field test CSV logs and DB data
- [ ] Migrate RPiNAS OS to Ubuntu 26.04 LTS

---
