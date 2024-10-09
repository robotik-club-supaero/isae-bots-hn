#!/bin/bash
cd /home/pi/isae-bots-hn-2024
make main INTERACTIVE="" CMD="source /app/install/setup.bash; ros2 launch scripts/remote_control.launch BR:=/dev/ttyBR; $SHELL"
