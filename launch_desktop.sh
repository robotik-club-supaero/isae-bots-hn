#!/bin/bash

# run term_installer (just in case the config has not been updated)
./dev/src/uix/log/term_installer.sh > /dev/null

# this command disables the authorization protocol so that 
# the interface handler (the X server) can be accessed inside docker
# It might cause some security problems but osef for now
#TODO maybe could use xhost +local:docker instead to only allow access to docker
xhost + > /dev/null

# launch the log interface
./dev/src/uix/log/log_launch.sh

