# NRF-supervised Wi-Fi manual-control field experiment

This experiment tests phone drive/steering control through tractor01 using a
dedicated extension to the Teensy firmware and Teensy bridge. It is deliberately
not an NRF24 replacement yet.

## Control path and limitations

```text
phone browser -> keyed field server on tractor01 -> localhost UDP 6004
              -> Teensy bridge -> WIFI serial message -> Teensy
```

- The physical handheld remains the authoritative mode selector.
- Put the handheld in **Auto** to authorize phone control.
- Put the handheld in **Pause** for the authoritative normal stop.
- NRF loss continues to force the existing Teensy radio-loss safety state.
- Phone drive demand is continuously bounded to `-100%..+100%` and mapped
  inside the Teensy to the proven handheld envelope: full reverse `3138`,
  exact neutral `2836`, and full forward `2288`.
- The dedicated `WIFI` serial message leaves ordinary autonomous `CMD`
  messages and their positive-forward m/s calibration unchanged.
- Steering is bounded to `-100%..+100%` and converted to the Teensy's existing
  normalized Auto steering convention.
- Phone commands are sent at 5 Hz with a single in-flight HTTP request,
  monotonically increasing sequence numbers, and full-state messages.
- The phone displays the rolling average of successful command round trips
  received during the most recent five seconds.
- The server accepts one phone owner, starts in Pause, rejects stale sequences,
  and requires a new guarded Manual selection after reconnection.
- The Teensy's existing 500 ms `cmd_vel` timeout remains the final command-loss
  watchdog. The field server also pauses after 800 ms without a fresh phone
  command.
- Do not run Pure Pursuit or another UDP 6004 publisher at the same time.

## Files

- `wifi_manual_control_field_server.py` — tractor01 server, validation, UDP
  output, telemetry merge, and JSONL logging.
- `wifi_manual_control_field.html` — landscape phone interface.
- The earlier `wifi_manual_control_test_server.py` remains a computer-only
  bench simulator and is not used on tractor01.

## Before leaving for the field

1. Commit and push the reviewed experiment from the development computer.
2. Pull it onto tractor01 using the normal `git pull --ff-only` procedure.
3. With the tractor unable to move, run the server once with `--dry-run` and
   verify that the phone displays real tractor telemetry but UDP 6004 is
   reported disabled.
4. Confirm the physical e-stop and handheld Pause work independently of the
   phone.

Dry-run command on tractor01:

```bash
cd /home/al/tractor2025
python3 tractor_rpi/testing/webrtc/wifi_manual_control_field_server.py --dry-run
```

## Field setup

1. Keep the mower blades off and begin with the tractor stationary in a large,
   clear area. The first powered test should be on stands or otherwise unable
   to propel itself.
2. Keep the physical handheld and physical emergency stop immediately
   available.
3. Select **handheld Pause** before starting any software.
4. Run the normal tractor preflight and require a pass:

   ```bash
   cd /home/al/tractor2025
   sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260926
   ```

5. Start the existing field logger in a separate terminal if it is not already
   started by the chosen field workflow:

   ```bash
   cd /home/al/tractor2025
   python3 tractor_rpi/field_test_logger_20260828.py
   ```

6. Do **not** start Pure Pursuit or the mission launcher.
7. Start the phone-control server:

   ```bash
   cd /home/al/tractor2025
   python3 tractor_rpi/testing/webrtc/wifi_manual_control_field_server.py
   ```

8. Open the printed URL containing the temporary `?key=...` value. Use the
   ZeroTier URL when remote or the local URL on the tractor field network.
   At startup the server also sends both URLs to the existing
   `rpi-tractor01-jones2126` ntfy topic; tapping the notification opens the
   ZeroTier URL. An ntfy delivery failure is only a warning and does not stop
   the server. Use `--no-ntfy` when a notification is not wanted, or
   `--ntfy-topic TOPIC` to select another topic.
9. The phone cannot claim control until fresh UDP 6003 telemetry confirms
   handheld Pause and a good NRF link.

## Driving sequence

1. Confirm the phone shows:
   - Handheld mode `PAUSE`
   - Phone mode `PAUSE`
   - Live phone link
   - Expected RTK and heading indicators
2. Set phone drive to `0%` and steering to `0%`.
3. Select **handheld Auto**.
4. On the phone, hold **MODE SELECT + Manual**. This arms phone commands; it
   does not change the physical handheld mode.
5. Begin with straight steering and a small positive drive demand.
6. Exercise small steering changes before larger ones.
7. To stop from the phone, hold **MODE SELECT + STOP** or select guarded phone
   Pause. Then select **handheld Pause**.
8. Before approaching the tractor, confirm physical handheld Pause in both the
   handheld switch and phone telemetry.

## Connection-loss test

Perform the first loss test with the tractor stationary or unable to propel
itself.

1. Establish phone Manual with the handheld in Auto.
2. Request `0%` drive and a visible steering offset.
3. Disable the phone network or ZeroTier.
4. Confirm phone commands expire, the Teensy reports `NO_CMD`, and steering
   drive stops after the existing 500 ms timeout.
5. Confirm reconnection remains in phone Pause and cannot resume automatically.
6. Return the handheld to physical Pause before further action.

Only after that passes should a moving loss test be considered at the lowest
useful forward speed in a clear area.

## Stop conditions

Immediately select physical handheld Pause, and use the physical emergency
stop when necessary, for any of the following:

- Unexpected motion or steering direction
- Phone commands while the interface says Pause
- More than one active controller
- Stale telemetry or incorrect handheld-mode indication
- Failure to stop within the expected command timeout
- Reconnection that restores motion without a new guarded Manual action
- Obstacle proximity, person/animal entry, or loss of operator sightline

Do not continue merely to complete the checklist.

## Logs and shutdown

The server writes an auto-named JSONL log under `/home/al/field_logs/` containing
claims, accepted/rejected sequences, stop bursts, decisions, and timestamps.
Preserve it together with the normal field logger CSV.

End the test in this order:

1. Set phone drive demand to neutral (`0%`).
2. Select guarded phone Pause/STOP.
3. Select physical handheld Pause.
4. Press `Ctrl+C` in the phone-server terminal; shutdown sends another neutral
   burst.
5. Stop the field logger cleanly and preserve both logs.
