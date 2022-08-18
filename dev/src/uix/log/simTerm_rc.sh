#!/bin/bash

source /opt/ros/noetic/setup.bash


# BINDKEYS



COM="ps -ef | grep /bin/roslaunch | awk 'NR==1{print \$2}'"
COMMAND3="Shutting down roslaunch"
bind -x '"\C-R":"echo $COMMAND3; kill -s SIGINT $(eval $COM)"'


COMMAND1="[OUTPUT COMMAND] -> Set side to HOME"
bind -x '"\C-H":"echo $COMMAND1; rostopic pub --once /game/color std_msgs/Int16 0 > /dev/null"'

COMMAND2="[OUTPUT COMMAND] -> Set side to AWAY"
bind -x '"\C-A":"echo $COMMAND2; rostopic pub --once /game/color std_msgs/Int16 1 > /dev/null"'

COMMAND0="[OUTPUT COMMAND] -> Start match"
bind -x '"\C-S":"echo $COMMAND0; rostopic pub --once /game/start std_msgs/Int16 1 > /dev/null"'


# INFO MESSAGE AT STARTUP

cols="$(tput cols)"

printf "%*s\n" $cols "La saucisse est bonne"

echo -e "####################################\n######  Simulation Terminal  #######\n####################################\n"

echo -e "\033[1mKey Commands :\033[0m\n\
Ctrl-H : set side to HOME\n\
Ctrl-A : set side to AWAY\n\
Ctrl-S : start match\n\
Ctrl-C : stop match\n\
Ctrl-D : clear terminal\n"