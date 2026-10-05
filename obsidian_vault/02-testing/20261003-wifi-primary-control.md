# 2026-10-03 Wi-Fi-primary control for Tractor01

## Outcome

This dated control path replaces the failed NRF24 link with the phone's Wi-Fi
data feed for low-level Manual drive and steering plus guarded Auto mission
authority. It preserves the existing physical E-stop circuit and keeps a
prominent guarded software STOP on the phone. The older NRF-supervised
experiment remains available unchanged.

This is a production candidate, not a declaration that an untested machine is
safe to drive. Complete the engine-off and wheels-raised checks below before
ground movement.

## Control behavior

- Phone commands run at 5 Hz through the existing UDP 6004 and serial bridge.
- The Teensy independently expires the feed after 500 ms. It commands exact
  transmission neutral and stops steering output at its current position.
- The field server also sends a Pause burst after 800 ms without a phone
  heartbeat.
- A temporary Wi-Fi interruption does **not** clear the selected phone Manual
  or Auto state. When the same heartbeat resumes, the Teensy resumes that
  authority automatically. Manual restores the current sliders; Auto resumes
  only with an independently fresh navigation `CMD` feed.
- A deliberate phone Pause, guarded STOP, hidden browser page, or server
  shutdown remains Pause and does not automatically re-arm Manual.
- The phone uses the full calibrated steering operating range (`191..885`),
  identical to the previous Wi-Fi test. It does not add a reduced field-test
  steering bound. The existing 20-count mechanical-stop margins remain.
- The physical E-stop logic in the proven firmware is not changed.
- Guarded phone **E-STOP** latches the same engine-kill relay while also
  forcing transmission neutral and steering Pause. Pressing guarded E-STOP a
  second time unlatches the relay and leaves the phone in Pause. Manual and
  Auto are rejected while the software E-stop latch is active.
- The software E-stop latch survives phone-link loss and a server restart. An
  ordinary Pause never clears it. The phone displays the latch state reported
  by the Teensy.
- NRF telemetry continues to be reported for diagnosis but is not a control
  prerequisite after the Wi-Fi feed is selected.
- The first valid phone command latches Wi-Fi-primary authority until the
  Teensy reboots. Navigation `CMD` packets control motion only while the phone
  is in guarded Auto; Manual restores its complete slider state after parsing
  and Pause ignores navigation motion commands.

## New files

- `tractor_teensy/src/teensy_main_20261003_wifi.cpp`
- `tractor_teensy/platformio.wifi-primary.ini`
- `tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py`
- `tractor_rpi/testing/webrtc/wifi_primary_control_20261003.html`
- `tractor_rpi/testing/webrtc/setup_wifi_control_https_20261003.sh`
- `tractor_rpi/testing/test_wifi_primary_control_20261003.py`

The backward-compatible optional phone-mode field was added to:

- `tractor_teensy/src/teensy_main_20260914.cpp`
- `tractor_rpi/teensy_serial_bridge_20260728.py`

The installed `teensy_serial_bridge_20261002.py` inherits that bridge change,
so its systemd service path does not need to change.

## Voice commands

First arm Manual on screen with **MODE SELECT + MANUAL**. Voice recognition can
then accept these exact patterns:

- `tractor forward 10` through `tractor forward 100`
- `tractor reverse 10` through `tractor reverse 100`
- `tractor left 10` through `tractor left 100`
- `tractor right 10` through `tractor right 100`
- `tractor neutral`
- `tractor straight`
- `tractor stop`, `tractor pause`, or `emergency stop`

Voice can never arm Manual. Stop/Pause voice phrases are accepted regardless
of the current mode. The screen speaks confirmation after a recognized command.

Phone microphone recognition may require a trusted HTTPS page. The server
supports `--tls-cert` and `--tls-key`; both files must be supplied together.
Do not treat voice as verified until the actual field phone shows **VOICE ON**,
recognizes the wake word, and passes the stationary tests. Sliders and the
guarded STOP remain available if that browser does not support recognition.

One-time trusted HTTPS setup on Tractor01:

```bash
cd /home/al/tractor2025
bash tractor_rpi/testing/webrtc/setup_wifi_control_https_20261003.sh
```

Copy only
`/home/al/.config/tractor-wifi-control/tls/tractor-wifi-control-root-ca.crt`
to the phone and install it as a trusted user CA. Never copy either `.key`
file. After that setup, start the server with:

```bash
python3 tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py \
  --tls-cert /home/al/.config/tractor-wifi-control/tls/server.crt \
  --tls-key /home/al/.config/tractor-wifi-control/tls/server.key
```

## Automatic boot service

The tracked `tractor-wifi-control.service` starts the HTTPS phone control
server after the network, ZeroTier daemon, and Teensy bridge have been ordered
at boot. The server immediately enters Pause and sends an explicit Pause burst.
Systemd uses SIGINT for a normal stop so the server's shutdown path also sends
Pause/neutral; an unexpected failure is still covered by the Teensy's
independent 500 ms heartbeat timeout.

The server sends separate ntfy notices for local and ZeroTier access. Local
readiness accepts any private IPv4 address on a non-loopback, non-ZeroTier
interface and uses the certificate-safe, DHCP-independent URL
`https://raspberrypi.local:8765/`. ZeroTier readiness requires the service to
be active, `192.168.193.76` on a `zt*` interface, and three successful TCP
connections to the always-on RPi5NAS at `192.168.193.217:22`. After an initial
90-second observation window it sends one delayed notice and continues
retrying; a later recovery gets its own notice. Loss of ZeroTier does not
interrupt working local control.

The installed service stores one private URL access key at
`/home/al/.config/tractor-wifi-control/operator.key` (mode 0600). The URL is
therefore stable across service restarts and reboots. The visible phone page
automatically establishes a new control session in Pause after the server
returns; it does not need a newly opened ntfy link. Hidden or older tabs stop
sending heartbeats and cannot compete with the visible control page. During
upgrade, the installer preserves the most recent valid key from the service
journal so the already-issued URL remains valid. A fresh installation creates
the key once. Delete the key and restart the service only when deliberate
access-key rotation is required.

Install and enable the service on Tractor01 with:

```bash
cd /home/al/tractor2025
sudo bash tractor_rpi/setup/install_tractor_wifi_control_service.sh
```

If a manually started server is running, stop it with Ctrl+C before starting
the service. Then:

```bash
sudo systemctl start tractor-wifi-control.service
systemctl status tractor-wifi-control.service --no-pager
sudo journalctl -u tractor-wifi-control.service --since "5 minutes ago" --no-pager
```

## Deploy to Tractor01

Tractor01 currently has the NRF reverse-test firmware loaded and the bridge is
inactive. Keep the engine off and make the tractor unable to move.

```bash
cd /home/al/tractor2025
git pull --ff-only origin main
sudo systemctl stop teensy-bridge.service

cd /home/al/tractor2025/tractor_teensy
pio run -c platformio.wifi-primary.ini
pio run -c platformio.wifi-primary.ini -t upload
```

Wait for `/dev/teensy` to return after the firmware's startup delay, then:

```bash
sleep 60
ls -l /dev/teensy
sudo systemctl start teensy-bridge.service
sleep 3
systemctl is-active teensy-bridge.service
sudo journalctl -u teensy-bridge.service --since "2 minutes ago" --no-pager -n 60
```

Confirm the exact firmware identity before starting phone control:

```bash
python3 - <<'PY'
import json, socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", 6003))
s.settimeout(5)
d = json.loads(s.recvfrom(65535)[0])
print("firmware:", d.get("system", {}).get("firmware"))
print("steering:", d.get("steering", {}))
print("transmission:", d.get("transmission", {}))
PY
```

The firmware must be `teensy_main_20261003_wifi_v3`. Before a phone claims
control, radio-loss mode 9 / `NO_SIG`, steering PWM 0, and neutral JRK target
are expected and safe.

## Engine-off test sequence

With the boot service installed, start or confirm the server:

```bash
cd /home/al/tractor2025
sudo systemctl start tractor-wifi-control.service
systemctl is-active tractor-wifi-control.service
```

1. Open the keyed URL from the local or verified-ZeroTier ntfy notice. It
   claims control in Pause.
2. Confirm the page shows the expected firmware and Pause.
3. Run the dated GPS/preflight while the page remains in Pause:

   ```bash
   cd /home/al/tractor2025
   sudo python3 tractor_rpi/testing/mission_preflight_20261002.py \
     --expected-firmware teensy_main_20261003_wifi_v3
   ```

4. With the engine off and wheels raised, arm **MODE SELECT + MANUAL**.
5. Test Straight, small left/right demands, then full calibrated left/right.
6. Test drive demands while observing JRK target movement.
7. Test guarded STOP and the unchanged physical E-stop separately.
8. With a nonzero steering demand and zero drive, disable phone Wi-Fi for more
   than one second. Confirm transmission neutral and steering output stops.
9. Restore Wi-Fi. Confirm the same Manual demand resumes automatically.
10. Say each voice command at zero drive and verify the displayed value before
    considering engine-on use.

Do not proceed to ground movement after any unexpected direction, failure to
pause within 500 ms, wrong firmware identity, active JRK error, steering fault,
or physical E-stop problem.

## Logs

The server creates
`/home/al/field_logs/wifi_primary_control_YYYYMMDD_HHMMSS.jsonl`. Each accepted
command records phone sequence, client timestamp, the phone's rolling RTT from
the preceding request, estimated phone-to-server time, and server processing
time. UDP 6003 also retains Teensy command age, steering response, and JRK
diagnostics for correlation with the normal field logger.

## 2026-10-05 boot-service and reconnect acceptance

Tractor01 pulled commit `c8b24e7` and re-ran the service installer. The
installer preserved the URL key already issued to the phone and stored it in
the private persistent key file. The service then stopped through its normal
Pause/neutral path and restarted active. Both local-ready and verified
ZeroTier-ready ntfy notices were sent.

With the newly served ZeroTier page open, a second service restart was tested
without opening a new ntfy link. The existing page displayed reconnecting for
approximately 0.5 seconds and automatically returned connected in Pause. This
verifies the intended stable-URL, fresh-session, and safe-reconnect behavior.
Repeated ntfy notices after future boots are readiness notices and should carry
the same URL. Keep one visible control tab; hidden tabs no longer contend for
control ownership.

See [[20261005-wifi-control-boundary-mission-handoff]] for the consolidated
field handover and pointers to the two new-chat context documents.
