#!/bin/sh
# wifi_publish_20260703.sh
# Runs ON the GL-SFT1200 router (OpenWrt). Polls `iwinfo` for whichever
# radio is currently connected to EXPECTED_SSID (matched by ESSID, not by
# interface name - confirmed via testing that the router renames its
# client-mode radio across reboots, e.g. wlan-sta0 -> sta1) and pushes a
# small JSON status line to the tractor RPi over TCP.
#
# CHANGED (7/3/26): caches the matched interface name and queries it directly
# on most cycles, instead of scanning all radios every time. Benchmarked on
# this router: `iwinfo <iface> info` (single radio) = ~2.5s, bare `iwinfo`
# (all 4 radios) = ~10s. Falls back to the full scan only when the cached
# interface stops matching EXPECTED_SSID (e.g. after a router reboot renames
# it), so it self-heals without needing a restart.
#
# Deploy: copy to /root/wifi_publish.sh on the router, chmod +x it, then
# either run it manually for testing or wire it into /etc/rc.local (or a
# proper /etc/init.d script) so it starts on boot. See notes at bottom.

# ---- Config - EDIT PER DEPLOYMENT ----
TARGET_IP="192.168.8.20"      # tractor RPi's LAN IP on eth0 (tractor02 - static DHCP reservation as of 7/3/26)
TARGET_PORT=6005              # new port; does not conflict with 6002/6003/6004
EXPECTED_SSID="TractorField"
POLL_INTERVAL=2               # seconds between pushes
# ---------------------------------------

CACHED_IFACE=""   # NEW (7/3/26): remembers which radio last matched EXPECTED_SSID

while true; do
    result=""

    # NEW (7/3/26): fast path - query only the cached interface directly (~2.5s)
    if [ -n "$CACHED_IFACE" ]; then
        result=$(iwinfo "$CACHED_IFACE" info 2>/dev/null | awk -v ssid="$EXPECTED_SSID" -v iface="$CACHED_IFACE" '
            /ESSID:/ {
                e=$0
                sub(/.*ESSID: "/, "", e)
                sub(/".*/, "", e)
                essid=e
            }
            /Signal:/ && essid==ssid {
                s=$0
                sub(/.*Signal: /, "", s)
                sub(/ dBm.*/, "", s)
                print iface":"essid":"s
                exit
            }
        ')
        if [ -z "$result" ]; then
            CACHED_IFACE=""   # no longer valid - force the slow path below to re-find it
        fi
    fi

    # SLOW path - only runs on first cycle, or after the cache misses (~10s)
    if [ -z "$result" ]; then
        result=$(iwinfo | awk -v ssid="$EXPECTED_SSID" '
            /^[^ \t]/ { iface=$1; essid="" }
            /ESSID:/ {
                e=$0
                sub(/.*ESSID: "/, "", e)
                sub(/".*/, "", e)
                essid=e
            }
            /Signal:/ && essid==ssid {
                s=$0
                sub(/.*Signal: /, "", s)
                sub(/ dBm.*/, "", s)
                print iface":"essid":"s
                exit
            }
        ')
        if [ -n "$result" ]; then
            CACHED_IFACE=$(echo "$result" | cut -d: -f1)   # NEW: remember it for next cycle
        fi
    fi

    if [ -n "$result" ]; then
        ssid=$(echo "$result" | cut -d: -f2)
        rssi=$(echo "$result" | cut -d: -f3)
        if [ "$rssi" -ge -65 ] 2>/dev/null; then
            label="strong"
        elif [ "$rssi" -ge -80 ] 2>/dev/null; then
            label="medium"
        else
            label="weak"
        fi
        msg="{\"wifi_ssid\":\"$ssid\",\"wifi_rssi_dbm\":$rssi,\"wifi_signal_label\":\"$label\"}"
    else
        msg="{\"wifi_ssid\":null,\"wifi_rssi_dbm\":null,\"wifi_signal_label\":\"unknown\"}"
    fi

    # CHANGED (7/3/26): this router's BusyBox nc has no -u/-w flags (confirmed
    # via testing: `nc` alone prints "Usage: nc [IPADDR PORT]") - TCP only.
    # Wrapped in `timeout` as a safety net in case the RPi side doesn't close
    # the connection promptly, so this loop can never hang indefinitely.
    if command -v timeout >/dev/null 2>&1; then
        printf '%s' "$msg" | timeout 3 nc "$TARGET_IP" "$TARGET_PORT" 2>/dev/null
    else
        printf '%s' "$msg" | nc "$TARGET_IP" "$TARGET_PORT" 2>/dev/null
    fi

    sleep "$POLL_INTERVAL"
done

# ---- To run persistently across router reboots ----
# Option A (quick, survives reboot on most GL-iNet firmware):
#   Add this line to /etc/rc.local, above the "exit 0" line:
#     /root/wifi_publish.sh &
#
# Option B (cleaner, proper OpenWrt service):
#   Create /etc/init.d/wifi_publish with a standard procd wrapper -
#   ask if you want this drafted once Option A is confirmed working.
