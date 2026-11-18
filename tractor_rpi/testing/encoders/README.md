# Wheel encoder reference supplied November 17

`wheel_encoder_status_20261117_reference.md` preserves Al's supplied bring-up
handoff. It is reference evidence, not installation/firmware/udev approval.
Statements about hardware and prior captures are operator/session reports;
the new position producer has not yet validated them in the field.

The combined capture candidate selects separate encoder USB devices by serial
identity and consumes the described ENC/WARN/ERR lines. It does not flash their
boards, apply the proposed udev rules, or assume the provisional wheel-distance
scale is calibrated. It must not claim the main control Teensy.
