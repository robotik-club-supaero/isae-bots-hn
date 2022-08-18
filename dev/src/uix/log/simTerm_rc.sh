#!/bin/bash

source /opt/ros/noetic/setup.bash

ROSLAUNCH_RUNNING=1

# BINDKEYS

COMMANDK="[SIM COMMAND] :  Shut down roslaunch"
COMMANDK2="[ERROR] No roslaunch is currently active"
COM="ps -ef | grep /bin/roslaunch | awk 'NR==1{print \$2}'"
bind -x '"\C-K":"if [[ $ROSLAUNCH_RUNNING == 1 ]]; then echo $COMMANDK; ROSLAUNCH_RUNNING=0; kill -s SIGINT $(eval $COM); else echo $COMMANDK2; fi"'


COMMANDR="[SIM COMMAND] :  Restart simulation"
COMMANDR2="[ERROR] Stop the roslaunch before restarting"
COM2="ps -ef | grep /bin/bash | awk '{if(\$3 == 1){print \$2}}'"
bind -x '"\C-R":"if [[ $ROSLAUNCH_RUNNING == 0 ]]; then echo $COMMANDR; ROSLAUNCH_RUNNING=1; kill -s SIGHUP $(eval $COM2); else echo $COMMANDR2; fi"'


COMMANDH="[ROS MESSAGE] -> Set side to HOME"
bind -x '"\C-H":"echo $COMMANDH; rostopic pub --once /game/color std_msgs/Int16 0 > /dev/null"'

COMMANDA="[ROS MESSAGE] -> Set side to AWAY"
bind -x '"\C-A":"echo $COMMANDA; rostopic pub --once /game/color std_msgs/Int16 1 > /dev/null"'

COMMANDG="[ROS MESSAGE] -> Start match"
bind -x '"\C-G":"echo $COMMANDG; rostopic pub --once /game/start std_msgs/Int16 1 > /dev/null"'


# INFO MESSAGE AT STARTUP

cols="$(tput cols)"
((cols=cols/2+13))
printf "%*s\n" $cols "#########################"
printf "%*s\n" $cols "#  Simulation Terminal  #"
printf "%*s\n" $cols "#########################"

echo -e "\033[1mKey Commands :\033[0m\n\
Ctrl-H : set side to HOME\n\
Ctrl-A : set side to AWAY\n\
Ctrl-G : start match\n\
\n\
Ctrl-D : clear terminal\n\
Ctrl-K : stop roslaunch\n\
Ctrl-R : restart the simulation\n\
Ctrl-Q : exit the simulation\n"