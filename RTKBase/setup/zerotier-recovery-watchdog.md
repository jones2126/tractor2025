# ZeroTier Recovery Watchdog

This service keeps remote access to the Bridgeville RTK base working after the
Starlink service or router has been powered down and later restored.

## Behavior

Every 60 seconds, the watchdog checks both of these conditions:

1. `zerotier-one.service` is running.
2. The always-on RPi5NAS responds at its ZeroTier address, `192.168.193.217`.

When both checks pass, the watchdog sleeps until the next check. If either
check fails, it enters recovery mode:

1. Wait for public HTTPS connectivity to return.
2. Restart `zerotier-one.service`.
3. Allow up to five minutes for the service to start and the NAS to respond.
4. Send an ntfy notice to `rpi-rtkbase-jones2126` after the NAS responds.

If the ZeroTier path does not recover, the watchdog waits five minutes and
tries the recovery cycle again. If ntfy is temporarily unavailable after the
NAS becomes reachable, notification delivery is retried every 30 seconds.

No notification is sent during normal healthy checks. One notification is sent
for each outage/recovery event.

## Install on the RTK base

First, pull the current repository on the base station. Then run:

```bash
python3 -m py_compile \
  /home/al/tractor2025/RTKBase/Bridgeville/zerotier_recovery_watchdog.py

sudo install -o root -g root -m 0644 \
  /home/al/tractor2025/RTKBase/setup/zerotier-recovery-watchdog.service \
  /etc/systemd/system/zerotier-recovery-watchdog.service

sudo systemd-analyze verify \
  /etc/systemd/system/zerotier-recovery-watchdog.service
sudo systemctl daemon-reload
sudo systemctl enable --now zerotier-recovery-watchdog.service
```

The service runs as root because restarting `zerotier-one.service` requires
root permission. The watchdog does not require any extra Python packages.

## Confirm it is working

```bash
systemctl status zerotier-recovery-watchdog.service --no-pager
journalctl -u zerotier-recovery-watchdog.service -b --no-pager
```

The initial journal message should identify `192.168.193.217` as the peer being
watched. During normal operation there will be no message every minute; that is
intentional, so the journal stays quiet.

To follow recovery activity live:

```bash
journalctl -u zerotier-recovery-watchdog.service -f
```

Press `Ctrl+C` to stop following the journal. This does not stop the service.

## Safe recovery test

Do this only while you have local access to the RTK base, because the test
temporarily interrupts its ZeroTier connection:

```bash
sudo systemctl stop zerotier-one.service
```

The watchdog should detect the stopped service, confirm that the Internet is
available, restart ZeroTier, confirm the NAS is reachable, and send the ntfy
notice. Watch its progress with the journal command above.

## Change settings

Settings are kept in the repository service file. The most useful values are:

| Setting | Default | Purpose |
|---|---:|---|
| `NAS_ZEROTIER_IP` | `192.168.193.217` | Always-on NAS ZeroTier address |
| `NTFY_TOPIC` | `rpi-rtkbase-jones2126` | Recovery notification topic |
| `HEALTHY_CHECK_INTERVAL` | `60` | Seconds between healthy checks |
| `INTERNET_RETRY_INTERVAL` | `30` | Seconds between Internet checks |
| `RESTART_RETRY_INTERVAL` | `300` | Seconds before another restart cycle |

After changing the service file, install it again and run:

```bash
sudo systemctl daemon-reload
sudo systemctl restart zerotier-recovery-watchdog.service
```
