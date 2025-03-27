#!/bin/bash

# Function to cleanup
cleanup() {
    echo "Cleaning up HUD..."
    # Kill openbox
    pkill openbox
    # Kill X server
    pkill Xorg
    # Return to previous tty
    chvt 1
}

# Set trap for cleanup
trap cleanup EXIT

# Start X with a minimal window manager and run the HUD
export DISPLAY=:0
startx /usr/bin/openbox -- -nocursor &
sleep 3

# Run from the current user's home directory
cd /home/alex
python3 /home/alex/lidar_hud.py 