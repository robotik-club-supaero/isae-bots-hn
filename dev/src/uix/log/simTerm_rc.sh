#!/bin/bash

source /opt/ros/noetic/setup.bash


# BINDKEYS

TEXT="[OUTPUT COMMAND] -> Start match"
bind -x '"0":"echo $TEXT; rostopic pub --once /game/start std_msgs/Int16 1 > /dev/null"'


# INFO MESSAGE AT STARTUP

echo -e "####################################\n######  Simulation terminal  #######\n####################################\n"