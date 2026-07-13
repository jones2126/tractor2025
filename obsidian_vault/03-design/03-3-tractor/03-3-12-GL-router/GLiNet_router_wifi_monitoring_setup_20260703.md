# GL-iNet GL-SFT1200 Router Setup & WiFi Signal Monitoring

**Date:** 2026-07-03
**Context:** Field architecture for tractor RPi internet access + WiFi signal monitoring.

## Architecture

```
Tractor RPi5 --[Ethernet]--> GL-SFT1200 router --[WiFi]--> Pixel_4952 hotspot
```

The tractor RPi reaches the internet via a wired connection to the GL-SFT1200
travel router, which in turn connects wirelessly (client/repeater mode) to a
phone hotspot. The RPi's own `wlan0` is **not** part of this path in the field
deployment - it's only used directly on bench units (e.g. tractor02 connecting
to `cui_bono` at the desk). Because of this, WiFi signal quality that matters
in the field is the router's hotspot link, not the RPi's own radio - so
monitoring code must query the **router**, not `iwconfig wlan0` on the RPi.

## Router details

| Item | Value |
|---|---|
| Model | GL-iNet GL-SFT1200 |
| Firmware | OpenWrt-based |
| LAN IP | `192.168.8.1` |
| LAN subnet | `192.168.8.0/24` |
| SSH user | `root` |
| SSH password | same as web admin login |
| Hotspot SSID | `Pixel_4952` |
| Admin web UI | `http://192.168.8.1` |

## Known quirks (confirmed via testing, 2026-07-02/03)

- **SSH host key algorithm**: this router's SSH server (Dropbear) only offers
  `ssh-rsa`, which modern OpenSSH clients disable by default. Every `ssh`/`scp`
  command to this router needs:
  ```
  -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa
  ```
- **scp needs `-O`**: the router's SSH server doesn't support the newer
  SFTP-based transfer OpenSSH defaults to. Use the older SCP protocol:
  ```
  scp -O -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa <file> root@192.168.8.1:<path>
  ```
- **`nc` is BusyBox minimal - TCP only**: no `-u` (UDP) or `-w` (timeout)
  flags are supported. Usage is just `nc IPADDR PORT`. Any script that needs
  to push data off the router should use plain TCP, ideally wrapped in
  `timeout` as a safety net against a hung connection.
- **Client-mode WiFi interface name is NOT stable across reboots.** Observed
  names for the same physical hotspot connection across three different
  reboots: `wlan-sta0` (2.4GHz), `sta1` (5GHz), `sta0` (2.4GHz). Any script
  querying WiFi status must match by **ESSID**, never by a hardcoded
  interface name.
- **`iwinfo` timing**: querying a single known interface directly
  (`iwinfo <iface> info`) takes ~2.5s. Querying all radios with no argument
  (`iwinfo`, needed to find which interface currently matches a given SSID)
  takes ~10s. Scripts that poll frequently should cache the last-matched
  interface name and only fall back to the full scan when the cache stops
  matching (e.g. after a reboot renames the interface).
- **eth0 carrier flapping**: the RPi's `eth0` link to this router was
  observed repeatedly gaining/losing carrier (~15-20s cycles), preventing
  DHCP from ever completing. Rebooting the RPi resolved it; root cause not
  fully identified (possibly transient after router power-cycle). If seen
  again, check `networkctl status eth0` for repeated Gained/Lost carrier
  log lines before assuming a config problem - it may be a physical-layer
  issue needing a reboot or cable/port swap rather than a networking fix.
- **`chmod +x` inside a heredoc block can silently not take effect.** When
  creating `/etc/init.d/wifi_publish` via a single heredoc + chmod command
  block, the file wrote correctly but the executable bit did not get set.
  Always verify with `ls -l` after creating an OpenWrt init script, and run
  `chmod +x` as a separate, explicit follow-up command if needed.

## WiFi signal monitoring: `wifi_publish.sh`

**Location on router:** `/root/wifi_publish.sh`
**Source of truth:** currently only on the router's local flash + downloaded
copies - not yet committed to the `tractor2025` repo. **TODO:** add this to
the repo (e.g. a new `RTKBase/router/` or similar folder) with this doc
describing the manual deploy step, since the router doesn't run
`repo-sync.service` the way the RPis do.

### What it does
1. Runs `iwinfo` on the router (all radios, or just the cached one) to find
   whichever interface is currently connected to `Pixel_4952`
2. Builds a small JSON string: `{"wifi_ssid":..., "wifi_rssi_dbm":...,
   "wifi_signal_label":...}`
3. Sends it over a plain TCP connection to the tractor RPi's LAN IP
   (`192.168.8.112` for tractor02) on port `6005`
4. Repeats every ~2 seconds (actual cadence ~3-5s due to `iwinfo` query time)

### Config values to check per deployment
Inside `wifi_publish.sh`:
```
TARGET_IP="192.168.8.112"     # tractor RPi's LAN IP on eth0 - EDIT per unit
TARGET_PORT=6005
EXPECTED_SSID="Pixel_4952"
POLL_INTERVAL=2
```

### Deploying/updating the script
```bash
scp -O -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa \
    wifi_publish_YYYYMMDD.sh root@192.168.8.1:/root/wifi_publish.sh
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa root@192.168.8.1
chmod +x /root/wifi_publish.sh
```

### RPi-side test listener
`tractor_rpi/testing/router_wifi_tcp_listener_TESTING.py` - minimal TCP
server that prints whatever it receives on port 6005. Used to confirm the
router's script is sending correctly. **TODO:** replace with a proper
listener thread inside `field_test_logger`, matching the pattern already
used by `gps_listener()`/`status_listener()`.

## Persistence: running `wifi_publish.sh` on router boot

Implemented as a proper OpenWrt `init.d`/`procd` service so it auto-restarts
on crash and starts after networking is up.

**File:** `/etc/init.d/wifi_publish`
```sh
#!/bin/sh /etc/rc.common
START=99
STOP=10
USE_PROCD=1

start_service() {
    procd_open_instance
    procd_set_param command /root/wifi_publish.sh
    procd_set_param respawn ${respawn_threshold:-3600} ${respawn_timeout:-5} ${respawn_retry:-0}
    procd_set_param stdout 1
    procd_set_param stderr 1
    procd_close_instance
}
```

**Setup commands:**
```bash
chmod +x /etc/init.d/wifi_publish   # verify with `ls -l` - see chmod quirk above
/etc/init.d/wifi_publish enable
/etc/init.d/wifi_publish start
```

**Verify it's running:**
```bash
ps | grep wifi_publish
```
(OpenWrt's `procd`-managed services don't reliably support a `status`
subcommand - `ps` is the dependable check.)

**Verify data is flowing**, from the RPi:
```bash
python3 router_wifi_tcp_listener_TESTING.py
```

**Verify persistence across a reboot:**
```bash
reboot
```
Wait ~30-60s, then repeat the `ps` and listener checks - the service should
start itself with no manual intervention.

## Open items / not yet done

- [ ] Commit `wifi_publish.sh` and this doc into the `tractor2025` repo
- [ ] Replace `router_wifi_tcp_listener_TESTING.py` with a real listener
      thread inside `field_test_logger`, adding `router_wifi_ssid` /
      `router_wifi_rssi_dbm` / `router_wifi_signal_label` columns to its CSV
      output (already scaffolded in `field_test_logger_20260703.py`, but that
      version still uses the abandoned SSH-polling approach - needs updating
      to match the TCP push design here)
- [ ] Confirm root cause of the eth0 carrier-flapping incident, if it recurs
- [ ] Decide whether `Option A` (`/etc/rc.local`) is still needed as a
      fallback, or whether the `init.d` service fully replaces it
