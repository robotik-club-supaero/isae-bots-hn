#!/bin/bash


# Launch docker container
make main CMD="echo lol"
#PERIPH=$(find /dev -name ttyACM* 2>/dev/null)

#echo "Périphériques connectés:"
#echo $PERIPH

#source /opt/ros/noetic/setup.bash
#cd ~/pr/hn
#killall -9 roslaunch python
#source devel/setup.bash
#cd ../scripts

#export ROS_IP=192.168.43.12
#export ROS_MASTER_URI=http://192.168.43.12:11311

#roslaunch match.launch BR:="/dev/ttyBR" ACT:="/dev/ttyACT" LIDAR:="/dev/ttyLIDAR" | tee matchLog.log
# the tee command allows to save logs in the matchLog file while keeping the display in the terminal (stdout)
