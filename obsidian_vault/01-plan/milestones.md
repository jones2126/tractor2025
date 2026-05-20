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

## C — GPS RTK Onboard
*Status: NOT STARTED*
- [ ] RTK position + heading live
- [ ] RTCM corrections streaming
- [ ] Waypoint display

## D — Pure Pursuit Navigation
*Status: NOT STARTED*
- [ ] Waypoint following
- [ ] Stable steering tracking
- [ ] Acceptable cross-track error
