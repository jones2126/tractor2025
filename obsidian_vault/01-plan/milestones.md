# Milestones

## A — Radio Manual Control
*Status: IN PROGRESS*
- [x] NRF24 radio communication working (addresses "1Node"/"2Node")
- [x] Steering direction fixed (RPWM/LPWM swap corrected 2026-05-18)
- [x] Asymmetric pot mapping calibrated (RC: right=1, center=503, left=1024)
- [x] Transmission bucket system working (10 buckets, neutral=2985)
- [ ] Steering PID tuned (currently kp=1.0, ki=0, kd=0 — jerky)
- [ ] Transmission neutral confirmed via JRK position reader
- [ ] Manual drive test with tractor moving

## B — Web Teleoperation
*Status: NOT STARTED*
- [ ] Browser controls working
- [ ] Oak camera feed live
- [ ] Low-latency steering response

## C — GPS RTK Related
*Status:  STARTED*
- [ ] RTK position + heading live
	- [ ] Applied latest firmware from ArduSimple for X20D board.  The update rate is only 1Hz.  I do have a test python script the reads the heading
- [x] RTCM corrections streaming
- [ ] Waypoint display
- [ ] Issues to clear
	- [ ] Get firmware for 10Hz or better update rate
	- [ ] RTK Base RPi seems to have a power issue - likely the 12V to 5V converter needs replacing
	- [ ] Install the 3D printed holder for the X20D

## D — Pure Pursuit Navigation
*Status: NOT STARTED*
- [ ] Waypoint following
- [ ] Stable steering tracking
- [ ] Acceptable cross-track error

---

## E — Electronics Board (Permanent Mount)
*Status: IN PROGRESS*

### Layout & CAD
- [x] Set RPi as origin (0, 0) in Fusion 360
- [x] Confirm Fusion 360 layout matches [Google Sheets placement table](https://docs.google.com/spreadsheets/d/1ZwWvbn1DCp-QlVXrmnlaehEPqhmPX6NaZv12z5NnVFc/edit?gid=149021576#gid=149021576)
- [ ] Physical wiring dry-run — verify component positions are reachable given real cable bend radii
- [x] Add remaining components to Fusion 360 model: Buck converter #1, Buck converter #2, relay board, JRK G2

### Power Distribution
- [ ] Run 14 AWG from blade fuse block to IBT-2; install 20 A fuse
- [x] Place and secure Buck converter #1 (RPi power) — route USB-C output to RPi; measure run and buy correct-length cable
- [ ] Place and secure Buck converter #2 (USB hub power) — design/print retention shroud
- [ ] Verify both Buck converters have clearance for tight wire bends on the board

### Component Placement (finalize positions)
- [ ] Install and wire Relay board — confirm it doesn't block USB or Ethernet access to the RPi
- [ ] Install and wire JRK G2 motor controller
- [ ] Decide if you will have a PCB for the voltage divider and power monitoring for the IBT-2

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
