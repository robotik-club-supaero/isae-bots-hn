#!/bin/bash

source /opt/ros/noetic/setup.bash

echo $1
echo "m.name=='$1'"

rostopic echo --filter="m.name=='$1'" /rosout_agg/msg | grep -ve ---
# TODO: enlever les quotes