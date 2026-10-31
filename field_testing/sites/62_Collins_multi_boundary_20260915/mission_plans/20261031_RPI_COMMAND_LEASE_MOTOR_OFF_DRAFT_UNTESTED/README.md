# RPi command-lease motor-off package — draft, untested

This package prepares a temporary, engine-off Teensy firmware exercise. It
builds both the known Wi-Fi firmware rollback image and the new RPi command-
lease candidate, runs focused tests, and hashes both images.

The retired NRF handheld is not a command or session-claim dependency. The
candidate accepts a new RPi session only when its first valid frame is an
explicit zero Pause command. Phone and Wi-Fi safety remain RPi responsibilities.

It intentionally contains no automatic upload. `package.json` keeps both flash
and motion approval false. After separate approval, the candidate may be
temporarily flashed only for the Pause-only commissioning tool, followed by a
mandatory rollback during the same session.
