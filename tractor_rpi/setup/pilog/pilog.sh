#!/bin/bash
# Daily folder follows the continuous recorder's America/New_York date.
# Overrides support isolated verification without touching production logs/state.
LOGROOT=${PILOG_LOG_ROOT:-/home/al/field_logs}
export TZ=America/New_York
LOGDIR="$LOGROOT/$(date +%F)"
umask 022
STATE=${PILOG_STATE:-/run/pilog.current}        # lives in RAM, so it is cleared every boot
NTFY_URL=${PILOG_NTFY_URL:-https://ntfy.sh/rpi-tractor01-jones2126}
# These RAM files reset on reboot, independently of the CSV/day rollover.
PENDING="${STATE}.notice-pending"
SENT="${STATE}.notice-sent"
# A root-run power logger must not leave a new daily folder unwritable by al.
if [ ! -d "$LOGDIR" ]; then
  install -d -o al -g al -m 0750 "$LOGDIR" || exit 1
fi

v=$(vcgencmd pmic_read_adc | awk '/EXT5V_V/ {split($NF,a,"="); sub("V","",a[2]); print a[2]}')
thr=$(vcgencmd get_throttled | cut -d= -f2)
read -r l1 l5 _ < /proc/loadavg
mhz=$(( $(vcgencmd measure_clock arm | cut -d= -f2) / 1000000 ))
temp=$(vcgencmd measure_temp | sed "s/temp=//; s/'C//")
up=$(cut -d' ' -f1 /proc/uptime)
now=$(date '+%F %T')

first=0
previous=""
if [ ! -s "$STATE" ]; then
  first=1
else
  previous=$(cat "$STATE")
fi

# Re-evaluate the date on EVERY timer sample. A new day or migration from the
# legacy directory creates a new file, but must not repeat the boot notice.
if [ "$first" = 1 ] || [ "$(dirname "$previous")" != "$LOGDIR" ] || [ ! -f "$previous" ]; then
  LOGFILE="$LOGDIR/pilog_$(date +%Y-%m-%d_%H%M%S).csv"
  if [ ! -e "$LOGFILE" ]; then
    echo "time,uptime_s,ext5v_v,throttled,load1,load5,arm_mhz,temp_c" > "$LOGFILE" || exit 1
  fi
else
  LOGFILE="$previous"
fi
printf '%s\n' "$LOGFILE" > "$STATE" || exit 1

echo "$now,$up,$v,$thr,$l1,$l5,$mhz,$temp" >> "$LOGFILE" || exit 1
sync "$LOGFILE" || exit 1

# Keep the first report until ntfy accepts it. Existing CSV state does not
# imply notification success (including an upgrade during the current boot).
if [ ! -s "$SENT" ] && [ ! -s "$PENDING" ]; then
  t=$(( thr ))
  uv_now=no;  (( t & 0x1 ))     && uv_now=YES
  uv_boot=no; (( t & 0x10000 )) && uv_boot=YES
  th_now=no;  (( t & 0x4 ))     && th_now=YES
  th_boot=no; (( t & 0x40000 )) && th_boot=YES

  prio=default; tags=white_check_mark
  if [ "$uv_now" = YES ] || [ "$uv_boot" = YES ]; then prio=high; tags=warning; fi

  msg="rpi-tractor01 boot power report
Sample recorded at $now (America/New_York)
5V rail: ${v} V
Under-voltage: now=$uv_now, since boot=$uv_boot
Throttled: now=$th_now, since boot=$th_boot (raw $thr)
Load: $l1  Clock: ${mhz} MHz  Temp: ${temp} C
Log: $LOGFILE"

  # Preserve the original sample, priority and message for delayed delivery.
  printf '%s\n%s\n%s\n' "$prio" "$tags" "$msg" > "${PENDING}.tmp" || exit 1
  mv "${PENDING}.tmp" "$PENDING" || exit 1
fi

if [ ! -s "$SENT" ] && [ -s "$PENDING" ]; then
  { IFS= read -r prio; IFS= read -r tags; } < "$PENDING"
  msg=$(tail -n +3 "$PENDING")
  # One bounded attempt per timer sample. A network failure or non-2xx HTTP
  # response leaves the report pending for the next two-minute invocation.
  if status=$(curl --silent --show-error --fail --connect-timeout 5 --max-time 10 \
      --output /dev/null --write-out '%{http_code}' \
      -H "Title: rpi-tractor01 boot" -H "Priority: $prio" -H "Tags: $tags" \
      -d "$msg" "$NTFY_URL") && [[ "$status" =~ ^2[0-9][0-9]$ ]]; then
    printf '%s\n' "$now" > "${SENT}.tmp" && mv "${SENT}.tmp" "$SENT" || exit 1
    rm -f -- "$PENDING"
    echo "pilog: boot power notice accepted by ntfy (HTTP $status); phone delivery is not confirmed."
  else
    echo "pilog: boot power notice pending (HTTP ${status:-unavailable}); retry on next timer sample." >&2
  fi
fi
