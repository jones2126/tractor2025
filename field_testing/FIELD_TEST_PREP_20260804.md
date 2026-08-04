# Tractor01 field preparation — 2026-08-04

Keep the tractor stationary, blades disengaged, transmission in Pause, and the
front wheels clear of people and obstacles throughout flashing and verification.

## 1. Update the one tractor repository

```bash
cd /home/al/tractor2025
git status --short
git pull --ff-only origin main
git rev-parse --short HEAD
```

Stop if `git status --short` reports unexpected local changes or the pull does
not fast-forward.

## 2. Stop software that owns the Teensy serial port

```bash
pgrep -af 'pure_pursuit_controller|field_test_logger'
sudo systemctl stop teensy-bridge.service
systemctl is-active teensy-bridge.service
```

The final command must report `inactive` before uploading.

## 3. Build and flash the new Teensy firmware

```bash
cd /home/al/tractor2025/tractor_teensy

PIO="$(command -v pio || true)"
if [ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO="$HOME/.platformio/penv/bin/pio"
fi
test -n "$PIO" || { echo 'ERROR: PlatformIO was not found.'; exit 1; }

"$PIO" run -e teensy41
"$PIO" run -e teensy41 -t upload
```

The selected build source is `teensy_main_20260804.cpp`. Its steering control,
PID calculation, PWM decision, and complete steering telemetry all run on the
same 50 ms / 20 Hz cycle.

If Teensy Loader requests the program button, press it once. Do not continue
unless the upload reports success.

## 4. Restart the bridge and prove the telemetry rate

```bash
sudo systemctl start teensy-bridge.service
sudo systemctl status teensy-bridge.service --no-pager -l

cd /home/al/tractor2025
python3 tractor_rpi/testing/verify_steering_telemetry_20260804.py --seconds 8
```

Required result:

```text
Measured rate: approximately 20 Hz
Median Teensy dt: approximately 50 ms
Missing fields: none
PASS: complete source-sequenced steering telemetry is arriving at 20 Hz.
```

Do not start the mission if sequence remains zero, state is `UNKNOWN`, the
measured rate is below 18 Hz, or the verifier reports missing fields.

## 5. Measure the physical straight-wheel pot value

Before changing any calibration constant:

1. Put the tractor in Manual long enough to position the front wheels visually
   straight, then return it to Pause.
2. Run the verifier again and note `current` from several UDP packets, or use
   the command below.

```bash
python3 - <<'PY'
import json, socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if hasattr(socket, 'SO_REUSEPORT'):
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
s.bind(('', 6003))
seen = set()
while len(seen) < 20:
    msg = json.loads(s.recvfrom(8192)[0])
    st = msg.get('steering', {})
    q = st.get('sequence')
    if q and q not in seen:
        seen.add(q)
        print(q, st.get('current'), st.get('setpoint'), st.get('state'))
PY
```

Photograph the physically straight wheels and save the 20 readings. Do not
change `STEER_POT_CENTER` in the field solely from the previous run's inference.

## 6. Short stationary command-path check

With the front axle safely unloaded or with ample clearance, run the existing
jackstand steering test and confirm that requested direction, pot movement,
LPWM/RPWM direction, and increasing `q` agree. Stop immediately for reversed
motion, sustained PWM without pot movement, or an IBT-2 latch.

## 7. Start the Collins mission and logger

The site launcher now uses `field_test_logger_20260804.py`, configured for the
20 Hz steering sequence.

```bash
cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1
./run_62_Collins_polygon_1_mission.sh
```

Begin in Pause. Confirm RTK Fixed, valid heading, telemetry PASS, and correct
mission/start position before selecting Auto.

## 8. Early stop and data preservation

If anything looks wrong, select Manual or Pause first, then stop the launcher
with Ctrl+C so both logs close cleanly. Before rebooting, record the two paths
printed by the launcher and copy both the field log and pursuit log to Windows.
