#!/bin/bash

make main CMD="roslaunch scripts/gr_desktop.launch"


bash -c "./dev/src/uix/log/log_launch.sh" > test.log

