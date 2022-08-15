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

# Basic entrypoint for ROS / Catkin Docker containers

# Setup ros environment
source /opt/ros/noetic/setup.bash


# Make the node files executable
chmod +x ./dev/src/stratgr/act/act_node.py \
         ./dev/src/stratgr/dec/dec_node.py \
         ./dev/src/stratpr/act/act_node.py \
         ./dev/src/stratpr/dec/dec_node.py \


# Make the script to display logs executable
chmod +x ./dev/src/uix/log/echo_logs.sh


if [ -n "$WDIR" ]; then 
    if [ ! -e "$WDIR/devel/setup.bash" ]; then
        cdir=$(pwd)
        cd $WDIR
        catkin_make
        cd cdir
    fi
    source "$WDIR/devel/setup.bash"
fi

# Setup environment variables
# export ...
# export ...

# Execute command passed to entrypoint
bash -c "$@"