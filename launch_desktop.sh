#!/bin/bash

# run term_installer (just in case the config has not been updated)
# pass the first argument in case we want a particular launch file

./dev/src/uix/log/term_installer.sh $1
RES=$?

if (( $RES == 1 )); then
    echo "Error in the terminal configuration"
    exit 1
fi

# this command disables the authorization protocol so that 
# the interface handler (the X server) can be accessed inside docker
# It might cause some security problems but osef for now
#TODO maybe could use xhost +local:docker instead to only allow access to docker
xhost + > /dev/null

xdg-open http://localhost:5000

# launch the log interface
./dev/src/uix/log/log_launch.sh

