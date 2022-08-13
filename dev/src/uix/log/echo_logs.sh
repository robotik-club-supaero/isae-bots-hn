#!/bin/bash

source /opt/ros/noetic/setup.bash

rostopic echo --filter="m.name=='$1'" /rosout_agg/msg | grep -ve ---
# TODO: enlever les quotes au debut et a la fin du message