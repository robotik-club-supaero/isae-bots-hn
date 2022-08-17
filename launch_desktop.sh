#!/bin/bash

#source /opt/ros/noetic/setup.bash

cd dev/
source devel/setup.bash
cd ../scripts

export ROS_IP=10.3.141.12
export ROS_MASTER_URI=http://10.3.141.12:11311


#python ./lcdLogs.py &
# background process to display logs during setup, killed by a SIGINT via a handler


#TODO : launch a script or command to make all node files executable

roslaunch gr_desktop.launch
