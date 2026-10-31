# RPi command-lease phone integration — motor off, draft, untested

This disabled package tests the authenticated direct-router phone page through
UDP 6004, the RPi Pause-only safety runtime, the generic `RPICMD` adapter, and
the Teensy v2 command-lease acknowledgement. It installs and enables no unit.

The live bridge accepts only an explicit zero Pause packet. Manual, Auto, Pure
Pursuit, nonzero, malformed, and non-finite packets are rejected before the
serial adapter. The retired NRF handheld is not referenced as an authority.

This package does not authorize motion. Candidate firmware rollback remains
mandatory during the same session.
