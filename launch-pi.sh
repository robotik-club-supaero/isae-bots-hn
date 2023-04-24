#!/bin/bash



#python ./lcdLogs.py &
# background process to display logs during setup, killed by a SIGINT via a handler



make main CMD="roslaunch scripts/gr_pi.launch"