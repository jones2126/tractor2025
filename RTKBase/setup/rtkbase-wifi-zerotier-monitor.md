# RTK Base Wi-Fi and ZeroTier Monitor

## Purpose

The RTK Base uses a Python monitoring script and a `systemd` service to:

1. Wait for the Raspberry Pi to obtain a usable IPv4 address on Ethernet or Wi-Fi.
2. Confirm that an `ntfy.sh` notification can actually be delivered.
3. Send a **local network ready** notification.
4. Verify that the ZeroTier data path is operational by connecting to an always-on ZeroTier peer.
5. Send a separate **ZeroTier ready** notification.
6. Log Wi-Fi signal strength every five minutes.
7. Notify when the Wi-Fi SSID changes or the Wi-Fi signal becomes weak.

This monitoring is intended for the Bridgeville RTK Base Station.

---

## Files

### Python monitoring script

Installed location:

```text
/home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
```

### Active systemd service

Installed location:

```text
/etc/systemd/system/wifi-monitor.service
```

### Recommended repository copy of the service

Keep a copy in the Git repository for rebuilding the RTK Base:

```text
/home/al/tractor2025/RTKBase/setup/wifi-monitor.service
```

### Wi-Fi RSSI log

```text
/home/al/tractor2025/RTKBase/Bridgeville/wifi_rssi_log.csv
```

### Current GPS position source

The monitor reads the latest location from:

```text
/home/al/tractor2025/RTKBase/Bridgeville/current_position.json
```

This JSON file is written by the RTK base server.

### Per-boot notification state

```text
/home/al/.cache/wifi-monitor/local-network.boot_id
/home/al/.cache/wifi-monitor/zerotier.boot_id
```

These files contain the Linux boot ID and prevent duplicate startup notifications when the service is restarted during the same Raspberry Pi boot.

---

## systemd service configuration

The active service is:

```ini
[Unit]
Description=WiFi and ZeroTier Monitor for RTK Base Station
Wants=zerotier-one.service
After=network.target zerotier-one.service

[Service]
Type=simple
User=al
Group=al
WorkingDirectory=/home/al/tractor2025/RTKBase/Bridgeville

Environment=ZEROTIER_TEST_HOST=192.168.193.217
Environment=ZEROTIER_TEST_PORT=22

ExecStart=/usr/bin/python3 -u /home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
Restart=on-failure
RestartSec=30
TimeoutStopSec=10

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### ZeroTier test peer

The configured always-on peer is the Raspberry Pi NAS:

```text
ZeroTier IP: 192.168.193.217
TCP test port: 22
```

The monitor treats ZeroTier as operational only when it can establish a TCP connection from the RTK Base ZeroTier address to SSH port 22 on the RPi NAS.

---

## Startup sequence

### 1. Service startup

At boot, `systemd` starts `wifi-monitor.service` after the basic network target and the ZeroTier service have been started.

The Python process initially waits:

```text
60 seconds
```

The script performs its own readiness checks. It does not assume that `network.target` means the Internet or ZeroTier is usable.

### 2. Local network readiness check

After the initial 60-second delay, the script checks these interfaces:

```text
eth0
wlan0
```

It looks for a usable, non-loopback, non-link-local IPv4 address.

If neither interface has an IPv4 address, the script waits 30 seconds and checks again.

When at least one interface has an address, the script attempts to send an `ntfy.sh` notification. A local IP address alone is not considered sufficient. The `curl` request to `ntfy.sh` must also succeed.

If notification delivery fails, the script waits 30 seconds and retries.

The notification title is:

```text
rtkbase local network ready
```

The message includes:

- IPv4 address for `eth0`
- IPv4 address for `wlan0`
- Connected Wi-Fi SSID
- Wi-Fi RSSI in dBm
- Signal label
- Current RTK base latitude and longitude, when available
- Google Maps location URL, when available

After successful notification delivery, the script writes:

```text
/home/al/.cache/wifi-monitor/local-network.boot_id
```

### 3. ZeroTier readiness check

After the local-network notification succeeds, the ZeroTier check runs in a background thread. Wi-Fi logging can continue while ZeroTier is still becoming operational.

The ZeroTier test repeats every:

```text
30 seconds
```

All of the following conditions must pass:

1. `zerotier-one.service` is active.
2. A network interface whose name begins with `zt` has an IPv4 address.
3. The route to `192.168.193.217` uses that ZeroTier interface.
4. SSH is listening on port 22 of the RTK Base ZeroTier address.
5. A TCP connection succeeds from the RTK Base ZeroTier address to:

```text
192.168.193.217:22
```

When all tests pass and the notification is successfully delivered, the notification title is:

```text
rtkbase ZeroTier ready
```

The notification includes:

- ZeroTier interface name
- RTK Base ZeroTier IPv4 address
- Confirmation that local SSH is listening
- Confirmation that the RPi NAS SSH port is reachable through ZeroTier

After successful delivery, the script writes:

```text
/home/al/.cache/wifi-monitor/zerotier.boot_id
```

### Why a ZeroTier address alone is not sufficient

The ZeroTier interface can receive its address before peer-to-peer traffic is fully usable. The active TCP connection to the RPi NAS verifies the ZeroTier data path rather than merely checking whether an address has been assigned.

This test proves that the RTK Base can reach the NAS through ZeroTier. It does not literally originate a PuTTY connection from a third remote computer, but it is a substantially stronger readiness test than checking the ZeroTier interface alone.

---

## Ongoing Wi-Fi monitoring

After the local-network startup notification, the script enters its normal Wi-Fi monitoring loop.

### Poll interval

```text
300 seconds
```

This is one sample every five minutes.

### CSV columns

The RSSI log contains:

```text
Timestamp
SSID
RSSI_dBm
Signal_Label
```

### Retention

The script retains the most recent:

```text
7 days
```

The CSV is pruned during each five-minute cycle.

Malformed CSV rows are retained rather than silently deleted.

---

## Signal labels

The monitor assigns these labels:

| RSSI | Label |
|---|---|
| `-65 dBm` or stronger | strong |
| Below `-65 dBm` through `-80 dBm` | medium |
| Below `-80 dBm` | weak |
| RSSI unavailable | unknown |

The weak-alert threshold is:

```text
-80 dBm
```

A weak notification is generated only when RSSI is strictly below `-80 dBm`.

---

## SSID change notifications

The script compares the current SSID with the SSID from the preceding five-minute sample.

When the SSID changes, it sends:

```text
rtkbase WiFi changed
```

The message includes the previous SSID, the current SSID or disconnected state, and current RSSI information.

No change notification is sent for the first sample after the script starts because there is no preceding SSID value for comparison.

---

## Weak-signal notifications

When RSSI drops below `-80 dBm`, the script sends:

```text
rtkbase WiFi weak signal
```

Only one notification is sent during a continuous weak-signal episode.

The weak-alert state resets when:

- RSSI recovers to `-80 dBm` or stronger, or
- The SSID changes.

A later drop below the threshold can then generate a new notification.

---

## ntfy.sh configuration

The topic is configured inside the Python script:

```python
NTFY_TOPIC = "rpi-rtkbase-jones2126"
```

The destination is:

```text
https://ntfy.sh/rpi-rtkbase-jones2126
```

The script uses `curl` with failure detection. A notification is marked as sent only when `curl` returns successfully.

Startup notifications retry after delivery failures. Routine SSID and weak-signal alerts are attempted during their normal monitoring cycle.

---

## GPS position handling

The script reads:

```text
/home/al/tractor2025/RTKBase/Bridgeville/current_position.json
```

Expected keys:

```json
{
  "lat": 40.3453512,
  "lon": -80.1288493
}
```

If the file is missing, malformed, or does not contain valid coordinates, the notification states:

```text
Location: unavailable
```

A missing position does not prevent a network notification.

---

## Service management commands

### Check status

```bash
systemctl status wifi-monitor.service --no-pager
```

### Check whether it is enabled at boot

```bash
systemctl is-enabled wifi-monitor.service
```

Expected:

```text
enabled
```

### View messages from the current boot

```bash
journalctl -u wifi-monitor.service -b --no-pager
```

### Follow messages live

```bash
journalctl -u wifi-monitor.service -b -f
```

Exit the live journal with `Ctrl+C`. This does not stop the service.

### Restart

```bash
sudo systemctl restart wifi-monitor.service
```

### Stop

```bash
sudo systemctl stop wifi-monitor.service
```

### Start

```bash
sudo systemctl start wifi-monitor.service
```

---

## Testing notifications again during the same boot

Normally the two startup notifications are sent only once per Linux boot.

To repeat the startup tests without rebooting:

```bash
rm -f /home/al/.cache/wifi-monitor/local-network.boot_id
rm -f /home/al/.cache/wifi-monitor/zerotier.boot_id
sudo systemctl restart wifi-monitor.service
```

The script will again wait 60 seconds before starting the local-network check.

---

## Useful diagnostic commands

### Show Ethernet and Wi-Fi addresses

```bash
ip -4 address show eth0
ip -4 address show wlan0
```

### Show the ZeroTier interface and address

```bash
ip -4 -o addr show | grep ' zt'
```

### Confirm the route to the RPi NAS

```bash
ip -4 route get 192.168.193.217
```

The result should show a ZeroTier interface whose name begins with `zt`.

### Confirm ZeroTier service status

```bash
systemctl is-active zerotier-one
```

Expected:

```text
active
```

### Confirm SSH is listening

```bash
sudo ss -ltnp | grep ':22'
```

### Test the NAS SSH port from the RTK Base

```bash
python3 - <<'PY'
import socket

source_ip = "192.168.193.88"
destination_ip = "192.168.193.217"
destination_port = 22

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.settimeout(5)
    sock.bind((source_ip, 0))
    result = sock.connect_ex((destination_ip, destination_port))

if result == 0:
    print("SUCCESS: NAS SSH is reachable through ZeroTier")
else:
    print(f"NOT READY: TCP connection returned error {result}")
PY
```

If the RTK Base ZeroTier address changes, replace `192.168.193.88` with the current address shown by `ip address`.

---

## Installation or rebuild procedure

### 1. Install the Python script

```bash
sudo install \
  -o al \
  -g al \
  -m 0755 \
  /home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py \
  /home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
```

The command above mainly ensures the correct owner and permissions when the script is already in its repository location.

Check syntax:

```bash
python3 -m py_compile \
  /home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
```

No output indicates a successful syntax check.

### 2. Install the systemd service from the repository

```bash
sudo install \
  -o root \
  -g root \
  -m 0644 \
  /home/al/tractor2025/RTKBase/setup/wifi-monitor.service \
  /etc/systemd/system/wifi-monitor.service
```

### 3. Validate and activate the service

```bash
sudo systemd-analyze verify /etc/systemd/system/wifi-monitor.service
sudo systemctl daemon-reload
sudo systemctl enable wifi-monitor.service
sudo systemctl restart wifi-monitor.service
```

### 4. Confirm operation

```bash
systemctl status wifi-monitor.service --no-pager
journalctl -u wifi-monitor.service -b --no-pager | tail -50
```

---

## Files that must not be deleted

Do not delete these active or operational files:

```text
/etc/systemd/system/wifi-monitor.service
/home/al/tractor2025/RTKBase/Bridgeville/wifi_monitor_20260624.py
/home/al/tractor2025/RTKBase/Bridgeville/current_position.json
/home/al/tractor2025/RTKBase/Bridgeville/wifi_rssi_log.csv
```

The files under `/home/al/.cache/wifi-monitor/` may be deleted for testing, but deleting them causes the startup notifications to be eligible for resending during the current boot.

---

## Current design summary

The monitor distinguishes between three different states:

1. **An interface has an IP address**
2. **The Internet path is usable enough to deliver an ntfy notification**
3. **The ZeroTier overlay can carry real TCP traffic to another member**

This prevents an early ZeroTier address assignment from being mistaken for a fully usable remote-access path.
