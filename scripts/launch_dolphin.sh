#!/bin/bash
# Launch Dolphin and move it to the Sunshine virtual display

open -a Dolphin
sleep 3

# Read virtual display ID written by Sunshine
VD_ID=""
if [ -f /tmp/sunshine_vd_id ]; then
    VD_ID=$(cat /tmp/sunshine_vd_id)
fi

if [ -z "$VD_ID" ] || [ "$VD_ID" = "0" ]; then
    echo "No virtual display active, Dolphin stays on current display"
    exit 0
fi

# Get the virtual display's origin coordinates
SCRIPT_DIR="$(dirname "$(readlink -f "$0" 2>/dev/null || echo "$0")")"
GET_ORIGIN="/Volumes/T7/sunshine/Sunshine/build/get_display_origin"

if [ ! -x "$GET_ORIGIN" ]; then
    echo "get_display_origin not found at $GET_ORIGIN"
    exit 0
fi

BOUNDS=$($GET_ORIGIN "$VD_ID")
X=$(echo "$BOUNDS" | awk '{print $1}')
Y=$(echo "$BOUNDS" | awk '{print $2}')

if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "Could not get virtual display bounds for ID $VD_ID"
    exit 0
fi

echo "Moving Dolphin to virtual display $VD_ID at ($X, $Y)"

# Move Dolphin window to the virtual display and make it fullscreen
osascript -e "tell application \"System Events\" to tell process \"Dolphin\" to set position of window 1 to {${X}, ${Y}}"
sleep 0.5
# Enter fullscreen
osascript -e 'tell application "System Events" to tell process "Dolphin" to keystroke "f" using {command down, shift down}'
