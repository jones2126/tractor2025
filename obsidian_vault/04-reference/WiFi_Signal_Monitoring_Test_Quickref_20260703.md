# WiFi Signal Monitoring — Test Quick Reference

**Purpose:** confirm a GL-SFT1200 router is correctly reporting its WiFi
link quality (SSID, RSSI, signal label) to a Raspberry Pi over the network,
using the disposable test listener script. Nothing here writes to a CSV or
database - this is purely "is the pipe working" verification.

**Current expected SSID:** `TractorField`
(earlier bench testing used `Pixel_4952` - the phone hotspot - before the
two-router field design settled on local AP name `TractorField`. If output ever shows `null` unexpectedly, check the
config note in Step 2 below first.)

---

## The two pieces involved

| File | Runs on | Job |
|---|---|---|
| `wifi_publish.sh` | The GL-SFT1200 router, at `/root/wifi_publish.sh` | Polls `iwinfo`, finds whichever radio is connected to the expected SSID, sends a small JSON status line over TCP |
| `router_wifi_tcp_listener_TESTING.py` | The Raspberry Pi | Listens on TCP port `6005`, prints whatever it receives, raw. No parsing, no logging - just proof the message arrived |

Transport is plain **TCP on port 6005** (not UDP - this router's BusyBox
`nc` build only supports bare TCP, no `-u`/`-w` flags, discovered the hard
way during initial testing).

---

## Step 1 — start the listener on the RPi first

```bash
cd ~/tractor2025/tractor_rpi/testing
python3 router_wifi_tcp_listener_TESTING.py
```

Expected immediately:
```
Listening for router WiFi status on TCP 6005... (Ctrl+C to stop)
```

Leave this running. It has to be listening *before* the router tries to
send anything, or the router will get "connection refused."

---

## Step 2 — SSH into the router and run the publisher script

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa root@<router-ip>
```
(The `-o HostKeyAlgorithms=...` flags are required every time - this
router's SSH server only offers the older `ssh-rsa` algorithm, which
modern SSH clients disable by default. Without these flags the connection
is refused outright before it even asks for a password.)

**Config checkpoint before running it:** open `/root/wifi_publish.sh` and
confirm this line matches the network you're actually testing against:
```bash
EXPECTED_SSID="TractorField"
```
If this doesn't match the real SSID currently broadcasting, every reading
will silently come back `null` - not a bug, just a mismatch.

Then run it:
```bash
/root/wifi_publish.sh
```
No output printed here by design - it runs silently in a loop, pushing a
message out over TCP every few seconds.

---

## Step 3 — watch the RPi terminal

You should start seeing lines like:
```
192.168.8.1 -> {"wifi_ssid":"TractorField","wifi_rssi_dbm":-52,"wifi_signal_label":"strong"}
```

**What "healthy" looks like:**
- A new line roughly every 3-5 seconds (first one may take up to ~10s - see
  the timing note below)
- `wifi_ssid` matches `TractorField` exactly
- `wifi_rssi_dbm` is a real negative number, reasonably stable between
  reads (small drift of a few dB is normal radio noise, not a problem)
- `wifi_signal_label`: `"strong"` (≥ -65 dBm), `"medium"` (≥ -80 dBm), or
  `"weak"` (below that)

**Why the very first line can be slow:** the script has to do a full scan
of all the router's radios once, to find which one currently matches
`EXPECTED_SSID` (~10 seconds). After that it caches the answer and queries
just that one radio directly, which is much faster (~2.5s). If the cached
radio ever stops matching (e.g. after a reboot renames it), expect one
slow ~10s cycle again as it re-locates the right radio - that's normal
self-healing behavior, not a fault.

---

## Step 4 — stop both sides

`Ctrl+C` in the router's SSH session, then `Ctrl+C` in the RPi's listener
terminal. Nothing persists from this test unless `wifi_publish.sh` is
separately running as a boot-time service (a different topic - see the
persistence section of the main router setup doc if that's what you're
checking).

---

## Quick troubleshooting reference

| Symptom | Likely cause |
|---|---|
| Nothing prints at all, no error either side | Confirm you're running the **TCP** listener, not an old UDP version. Check with `ss -tln \| grep 6005` on the RPi - should show `LISTEN 0.0.0.0:6005`. If it shows nothing, the wrong script is running or it crashed silently. |
| Router says `Connection refused` when testing manually | Nothing is listening on port 6005 on the RPi right now - start the listener first, or confirm no firewall (`sudo ufw status`) is blocking it. |
| SSH: `Unable to negotiate... Their offer: ssh-rsa` | Missing the `-o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa` flags on the `ssh` command. |
| `scp` fails to this router | Add `-O` to force the older SCP protocol - this router's SSH server doesn't support the newer SFTP-based transfer. |
| `nc: Usage: nc [IPADDR PORT]` | This router's BusyBox `nc` is TCP-only - don't use `-u` or `-w`, they don't exist on this build. |
| `wifi_ssid` always comes back `null` | Either the expected SSID isn't currently broadcasting/connected (check `iwinfo` directly on the router), or `EXPECTED_SSID` in the script doesn't match it exactly (case-sensitive). |
| Interface name looks different than last time you checked (`sta0` vs `sta1` vs `wlan-sta0`) | Expected - this router renames its client-mode radio across reboots. The script matches by SSID, not interface name, specifically to survive this. Nothing to fix. |
