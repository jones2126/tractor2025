# RPi command-lease motor-off package — draft, untested

This package prepares a temporary, engine-off Teensy firmware exercise. It
builds both the known Wi-Fi firmware rollback image and the new RPi command-
lease candidate, runs focused tests, and hashes both images.

It intentionally contains no automatic upload. `package.json` keeps both flash
and motion approval false. After separate approval, the candidate may be
temporarily flashed only for the Pause-only commissioning tool, followed by a
mandatory rollback during the same session.
