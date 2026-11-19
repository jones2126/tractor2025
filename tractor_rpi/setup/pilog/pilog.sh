#!/bin/bash
# Daily folder follows the continuous recorder's America/New_York date.
# Overrides support isolated verification without touching production logs/state.
LOGROOT=${PILOG_LOG_ROOT:-/home/al/field_logs}
export TZ=America/New_York
LOGDIR="$LOGROOT/$(date +%F)"
umask 022
STATE=${PILOG_STATE:-/run/pilog.current}        # lives in RAM, so it is cleared every boot
NTFY_URL=${PILOG_NTFY_URL:-https://ntfy.sh/rpi-tractor01-jones2126}
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

# Boot notification (first run after each boot only)
if [ "$first" = 1 ]; then
  t=$(( thr ))
  uv_now=no;  (( t & 0x1 ))     && uv_now=YES
  uv_boot=no; (( t & 0x10000 )) && uv_boot=YES
  th_now=no;  (( t & 0x4 ))     && th_now=YES
  th_boot=no; (( t & 0x40000 )) && th_boot=YES

  prio=default; tags=white_check_mark
  if [ "$uv_now" = YES ] || [ "$uv_boot" = YES ]; then prio=high; tags=warning; fi

  msg="rpi-tractor01 booted at $now
5V rail: ${v} V
Under-voltage: now=$uv_now, since boot=$uv_boot
Throttled: now=$th_now, since boot=$th_boot (raw $thr)
Load: $l1  Clock: ${mhz} MHz  Temp: ${temp} C
Log: $LOGFILE"

  curl -s --retry 5 --retry-delay 5 --retry-all-errors --retry-max-time 60 --max-time 20 \
    -H "Title: rpi-tractor01 boot" -H "Priority: $prio" -H "Tags: $tags" \
    -d "$msg" "$NTFY_URL" > /dev/null || true
fi
