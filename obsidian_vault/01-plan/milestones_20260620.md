# Milestones
*Last updated: 2026-06-20*

---

## 🔴 CRITICAL / Immediate Action Items

- [ ] **Field logger**: Get `field_test_logger_20260617.py` running on tractor02 (desk), then on tractor2025 (field). Root cause of last failure: no GPS data logged when base station was offline (RTCM failure blocked GPS parsing).
- [ ] **RTK base station**: Configure for field use from back of pickup — re-mount electronics, replace 12V→5V supply (RPi currently unstable).
- [ ] **X20D RTCM server**: Update `rtcm_server_x20d_20260615.py` to incorporate GPS speed tracking changes from `rtcm_server_20260617.py`.
- [ ] **tractor02 services**: Run `install_services.sh` (has not been run yet on tractor02). Then copy and enable services:
  ```bash
  sudo cp ~/tractor2025/tractor_rpi/setup/rtcm-server.service /etc/systemd/system/
  sudo cp ~/tractor2025/tractor_rpi/setup/teensy-bridge.service /etc/systemd/system/
  sudo cp ~/tractor2025/tractor_rpi/setup/led-controller.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable rtcm-server teensy-bridge led-controller
  sudo systemctl start rtcm-server teensy-bridge led-controller
  ```

---

## 🟡 Next Field Test Checklist

- [ ] Check for duplicate Netplan files on tractor02 — both `/etc/netplan/50-cloud-init.yaml` and `/etc/netplan/50-wifi.yaml` define `wlan0`; consolidate to one.
- [ ] Verify `.service` files copied from tractor2025 are correct for tractor02 (script paths, ports, device names may differ).
- [ ] Confirm field logger is capturing GPS data with base station present.
- [ ] Check voltage using watch command: $ watch -n 1 "vcgencmd pmic_read_adc | grep -i 5v"

---

## A — Radio Manual Control
*Status: IN PROGRESS*

- [x] NRF24 radio communication working (addresses "1Node"/"2Node")
- [x] Steering direction fixed (RPWM/LPWM swap corrected 2026-05-18)
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [x] Transmission bucket system working (10 buckets, neutral=2985)
- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky)
- [ ] Transmission neutral confirmed via JRK position reader
- [ ] Manual drive test with tractor moving

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

- [ ] RTK position + heading live
  - [ ] Apply latest ArduSimple firmware for X20D — currently only 1 Hz update rate; need 10 Hz
- [x] RTCM corrections streaming
- [ ] Waypoint display
- [ ] Decouple RTCM forwarding failure from GPS parsing — base station absence should not block GPS data (field test confirmed this bug)
- [ ] Issues to clear
  - [ ] Get firmware for 10 Hz or better update rate on X20D
  - [ ] RTK base station RPi power issue — replace 12V→5V converter (see CRITICAL above)
  - [ ] Install 3D-printed holder for X20D

---

## D — Pure Pursuit Navigation
*Status: NOT STARTED*

- [ ] Waypoint following
- [ ] Stable steering tracking
- [ ] Acceptable cross-track error

---

## E — Electronics Board - Gen 2 (Permanent Mount)
*Status: IN PROGRESS*

### Layout & CAD
- [x] Set RPi as origin (0, 0) in Fusion 360
- [x] Confirm Fusion 360 layout matches [Google Sheets placement table](https://docs.google.com/spreadsheets/d/1ZwWvbn1DCp-QlVXrmnlaehEPqhmPX6NaZv12z5NnVFc/edit?gid=149021576#gid=149021576)
- [x] Add remaining components to Fusion 360 model: Buck converter #1, Buck converter #2, relay board, JRK G2
- [ ] Physical wiring dry-run — verify component positions are reachable given real cable bend radii
- [ ] Make a custom PCB to hold Teensy 4.1 and NRF24 radio
- [ ] Laser cut an enclosure

### Power Distribution
- [x] Place and secure Buck converter #1 (RPi power) — route USB-C output to RPi
- [ ] Run 14 AWG from blade fuse block to IBT-2; install 20 A fuse
- [ ] Make PCB for RPi power pins for 20 pin header to be more durable
- [ ] Place and secure Buck converter #2 (USB hub power) — design/print retention shroud
- [ ] Verify both Buck converters have clearance for tight wire bends on the board

### Component Placement
- [ ] Install and wire relay board — confirm it doesn't block USB or Ethernet access to RPi
- [ ] Install and wire JRK G2 motor controller
- [ ] Decide if you will have a PCB for voltage divider and power monitoring for the IBT-2
- [ ] Fix LED andon signal light

### Pending Fabrications
- [ ] Print retention strap for Buck converters (reuse previously designed Fusion 360 model)
- [ ] Print retention shroud for kill-switch relay board
- [ ] Print bracket for JRK G2 controller

### Laser Cut Board
- [ ] Create Inkscape layout using finalized x, y coordinates from Fusion 360
- [ ] Obtain cut-file software used at Protohaven (Rabbit laser) — set up at home
- [ ] Cut board at Protohaven

### Final Assembly & Test
- [ ] Install standoffs
- [ ] Mount all components
- [ ] Wire and bench-test all components

---

## F — Data Infrastructure
*Status: NOT STARTED*

- [ ] Install PostgreSQL + TimescaleDB on RPiNAS (via apt or Docker)
  - Create hypertable with columns: `timestamp`, GPS lat/lon/alt/speed, sensor readings, device ID
  - Python listener on RPi that batches inserts from tractor UDP streams
  - Log video file paths and timestamps to DB
  - Add daily compression/backup cron jobs
- [ ] Install Grafana on RPiNAS to visualize field test CSV logs and DB data
- [ ] Migrate RPiNAS OS to Ubuntu 26.04 LTS

---

## G — Boot & Monitoring Improvements
*Status: NOT STARTED*

- [ ] Update `ntfy.sh` boot notification script to also publish:
  - `hostname -I` (all IP addresses)
  - `sudo zerotier-cli listnetworks` (ZeroTier network status)
