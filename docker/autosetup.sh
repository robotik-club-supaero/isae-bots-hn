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

# Make the scripts to display logs executable
chmod +x ./dev/src/uix/log/echo_logs.py
chmod +x ./dev/src/uix/log/simTerm_rc.sh
chmod +x ./dev/src/sim/sim/br/simulation_br

# Why is this necessary?
rm -rf /app/build/micro_ros_msgs/ament_cmake_python/micro_ros_msgs/micro_ros_msgs
colcon build --symlink-install
source "/app/install/setup.bash"

# Setup environment variables
# export ROS_IP=192.168.43.12
# export ROS_MASTER_URI=http://192.168.43.12:11311


# start bluetooth for remote control (pi only)
# service dbus start
bluetoothd &

# Execute command passed to entrypoint
bash -c "$@"