# NRF24 radio-loss incident — 2026-09-18

The post-main-backyard continuation reached a maximum controller waypoint of
1,821. At 378.820 seconds, the handheld state changed from `AUTO / OK` to
`mode 9 / NO_SIG`. The controller stopped driving immediately and froze the
recovery point at waypoint 1,798. It issued no driving cycles after the radio
loss.

The tractor position stream remained `RTK Fixed` for every available position
sample. The heading solution had intermittent invalid samples after the radio
loss while the tractor was stationary, but finished valid with fixed carrier.
RTCM forwarding, the GPS service, the Teensy bridge, and all USB device links
remained active during live diagnosis.

The dashboard, controller, and field logger were stopped cleanly before the
tractor power cycle. The final measured speed was 0.005 m/s. The archive
contains the two finalized CSV logs, the exact mission package, and snapshots
of the controller, dashboard, field logger, and preflight scripts.

Initial suspected causes are handheld battery voltage under load, handheld
NRF24 power or wiring, tractor NRF24 power or wiring, antenna connection, or a
module-level lockup. A clean Pi shutdown followed by a complete electronics
power cycle was selected as the next diagnostic step.
