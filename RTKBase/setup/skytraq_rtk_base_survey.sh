#!/usr/bin/env bash
# skytraq_rtk_base_survey.sh
# Interactive launcher for skytraq_rtk_base_survey_step_1.py
# Prompts for receiver type and survey duration, then runs the correct command.
# Place this file in tractor2025/RTKBase/setup/ alongside the Python scripts.
set -e
cd "$(dirname "$0")"

echo ""
echo "========================================"
echo "  SkyTraq RTK Base Survey Launcher"
echo "========================================"
echo ""

# --- Receiver selection ---
echo "Select receiver type:"
echo "  1) PX1172RH"
echo "  2) PX1125R"
echo ""
read -rp "Enter 1 or 2: " RECEIVER_CHOICE

case "$RECEIVER_CHOICE" in
    1)
        RECEIVER="PX1172RH"
        ;;
    2)
        RECEIVER="PX1125R"
        ;;
    *)
        echo "ERROR: Invalid selection. Enter 1 or 2."
        exit 1
        ;;
esac

echo ""

# --- Survey duration selection ---
echo "Select survey duration:"
echo "  1) 5-minute functional test  (indoor/bench, proves the workflow)"
echo "  2) 24-hour permanent base    (outdoor only, final antenna location)"
echo ""
read -rp "Enter 1 or 2: " DURATION_CHOICE

case "$DURATION_CHOICE" in
    1)
        DURATION_LABEL="5-minute test"
        ;;
    2)
        DURATION_LABEL="24-hour permanent base"
        ;;
    *)
        echo "ERROR: Invalid selection. Enter 1 or 2."
        exit 1
        ;;
esac

echo ""
echo "----------------------------------------"
echo "  Receiver : $RECEIVER"
echo "  Survey   : $DURATION_LABEL"
echo "----------------------------------------"

# Safety warning for 24-hour runs
if [ "$DURATION_CHOICE" = "2" ]; then
    echo ""
    echo "WARNING: 24-hour survey should only be run at the final outdoor"
    echo "antenna location, permanently mounted, with clear sky view."
    echo ""
    read -rp "Type YES to confirm the antenna is permanently mounted outdoors: " CONFIRM
    if [ "$CONFIRM" != "YES" ]; then
        echo "Aborted. Re-run when the antenna is in its final outdoor location."
        exit 1
    fi
fi

echo ""
echo "Starting survey..."
echo ""

# --- Build and run the command ---
if [ "$RECEIVER" = "PX1172RH" ] && [ "$DURATION_CHOICE" = "1" ]; then
    python3 skytraq_rtk_base_survey_step_1.py \
      --duration 300 \
      --label px1172rh-5min-test \
      --status-poll-seconds 20 \
      --post-survey-grace-seconds 300 \
      --minimum-monitor-timeout-seconds 900 \
      --max-stalled-polls 3 \
      --max-missing-status-polls 3

elif [ "$RECEIVER" = "PX1172RH" ] && [ "$DURATION_CHOICE" = "2" ]; then
    python3 skytraq_rtk_base_survey_step_1.py \
      --duration 86400 \
      --label px1172rh-permanent-base-24h \
      --status-poll-seconds 300 \
      --post-survey-grace-seconds 3600 \
      --minimum-monitor-timeout-seconds 90000 \
      --max-stalled-polls 3 \
      --max-missing-status-polls 3

elif [ "$RECEIVER" = "PX1125R" ] && [ "$DURATION_CHOICE" = "1" ]; then
    # --max-stalled-polls 100000 disables early countdown-stall abort
    # because PX1125R countdown behavior is not reliably monotonic
    python3 skytraq_rtk_base_survey_step_1.py \
      --duration 300 \
      --label px1125r-5min-test \
      --status-poll-seconds 20 \
      --post-survey-grace-seconds 300 \
      --minimum-monitor-timeout-seconds 900 \
      --max-stalled-polls 100000 \
      --max-missing-status-polls 3

elif [ "$RECEIVER" = "PX1125R" ] && [ "$DURATION_CHOICE" = "2" ]; then
    # --max-stalled-polls 100000 disables early countdown-stall abort
    python3 skytraq_rtk_base_survey_step_1.py \
      --duration 86400 \
      --label px1125r-permanent-base-24h \
      --status-poll-seconds 300 \
      --post-survey-grace-seconds 3600 \
      --minimum-monitor-timeout-seconds 90000 \
      --max-stalled-polls 100000 \
      --max-missing-status-polls 3
fi
