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
 
set -e

# This is a shell script that configures the build arguements to the Dockefile
# and builds a Docker image with a default tag.    
#              			  
# Available versions to configure the arguements:
# UBUNTU_RELEASE_YEAR = 18 and 20 
# ZED SDK Versions  = 3.0 to 3.7 (ZED SDK version <3 are too old and outdated)
# CUDA Versions = 10.0, 10.2, 11.0, 11.1, 11.4, 11.5
# ROS2_FLAG takes either 1 or 0 
#    1 sets the ROS_DISTRO_ARG to ROS2 Foxy Fitzroy
#    0 sets the ROS_DISTRO_ARG to ROS1 versions based on Ubuntu Release year