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

# Get the full path of this script
DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Get installed terminator (if not done already)
installed=$(dpkg -l | grep -E '^ii' | grep terminator)
if [ -z "$installed" ]
	then echo "Terminator is not installed yet, install it first ..."
	exit 1
	# else echo "Terminator is installed, moving on" && sleep 0.2
fi

# Replace config file
configure=~/.config/terminator/config
if [ -f "$configure" ]; then
    rm ~/.config/terminator/config
    # echo "Config file already existed, removed it"
else 
    echo "Config file didn't already exist"
	if ! [ -d "~/.config/terminator" ]; then
	mkdir ~/.config/terminator/
	echo "Created terminator config directory"
	fi
fi


# Copy term config file to keep the original
cp $DIR/term_simconfig ~/.config/terminator/config

# Check if no arguments were supplied
if [ "$#" -eq 0 ]
then
  	echo "No arguments supplied, using default launch file"
	sed -i "s/##LAUNCH_FILE##/gr_desktop.launch/g" ~/.config/terminator/config
else
	# check if file exists
	if [ -f "scripts/$1" ]; then
    	# replace default launch file by given launch file
		sed -i "s/##LAUNCH_FILE##/$1/g" ~/.config/terminator/config
		echo "Using lanch file scripts/$1"
	else 
		echo "$1 does not exist"
		exit 1
	fi
fi

# echo "Updated terminator configuration file"

# echo "All done !"
