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


# Make the node files executable
chmod +x ./dev/src/strat/act/act_node.py \
         ./dev/src/strat/dec/dec_node.py \
         ./dev/src/sensors/sensors_node.py \
         ./dev/src/sensors/lidar/lidar_node.py \
         ./dev/src/sensors/sonar/sonar_node.py \
         ./dev/src/isb/isb_node.py \
         ./dev/src/lcd/points_node.py



# Make the scripts to display logs executable
chmod +x ./dev/src/uix/log/echo_logs.sh
chmod +x ./dev/src/uix/log/simTerm_rc.sh

colcon build --symlink-install
source "$WDIR/install/setup.bash"

# Setup environment variables
# export ROS_IP=192.168.43.12
# export ROS_MASTER_URI=http://192.168.43.12:11311


# start bluetooth for remote control (pi only)
# service dbus start
bluetoothd &

# Execute command passed to entrypoint
bash -c "$@"