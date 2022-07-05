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

# This is a shell script that configures the build arguments of the Dockerfile
# and builds a Docker image with ROS and python 3.8

UBUNTU_RELEASE_YEAR=20  # DO NOT CHANGE !!

# Check for versions compatibilities

if [ ${UBUNTU_RELEASE_YEAR} == "18" ] ; then
	echo "Target OS: Ubuntu 18.04"
	ROS_DISTRO="melodic"
elif [ ${UBUNTU_RELEASE_YEAR} == "20" ] ; then	
    echo "Target OS: Ubuntu 20.04"
    ROS_DISTRO="noetic"
else
	echo "Wrong UBUNTU_RELEASE_YEAR... Allowed values are 18 or 20 "
	exit
fi

echo "Install for ROS 1 ${ROS_DISTRO}"
TAG="isaebot-ros-${ROS_DISTRO}-ubuntu${UBUNTU_RELEASE_YEAR}.04-desktop"
echo "Building ${TAG}"

docker build -t "$TAG" -f "Dockerfile.desktop" .