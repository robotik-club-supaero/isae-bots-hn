#!/bin/bash
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 
#

# Basic entrypoint for ROS / ament Docker containers

# Setup ros environment
source /opt/ros/jazzy/setup.bash

# HACK: urg_node2 hard-codes its configuration, so we need to change the source code...
# Comment the line below if we use an ethernet lidar
sed -i -e 's/ether/serial/g' /app/dev/lib/urg_node2/launch/urg_node2.launch.py

if cat /proc/cpuinfo | grep -iq "Raspberry"; then
# Use ttyLIDAR instead of ttyACM0 on Raspberry
sed -i -e 's/ACM0/LIDAR/g' /app/dev/lib/urg_node2/config/params_serial.yaml;
fi

# Why is this necessary?
rm -rf /app/build/micro_ros_msgs/ament_cmake_python/micro_ros_msgs/micro_ros_msgs
colcon build
source "/app/install/setup.bash"

# Setup environment variables
# export ROS_IP=192.168.43.12
# export ROS_MASTER_URI=http://192.168.43.12:11311


# start bluetooth for remote control (pi only)
# service dbus start
bluetoothd &

# Execute command passed to entrypoint
bash -c "$@"