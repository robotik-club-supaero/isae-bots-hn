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
	else echo "Terminator is installed, moving on" && sleep 1
fi

# Replace config file
configure=~/.config/terminator/config
if [ -f "$configure" ]; then
    rm ~/.config/terminator/config
    echo "Config file already existed, removed it"
else 
    echo "Config file didn't already exist"
	if ! [ -d "~/.config/terminator" ]; then
	mkdir ~/.config/terminator/
	echo "Created terminator config directory"
	fi
fi

cp $DIR/term_simconfig ~/.config/terminator/config
echo "Updated terminator configuration file"

# Ajout d'une fonction dans le bashrc
keybinded=$(grep control_bindkeys ~/.bashrc)

# Check si la fonction n'y est pas déjà pour ne pas l'avoir deux fois
if [ -z "$keybinded" ]
  then cat <<'EOF' >> ~/.bashrc

function control_bindkeys {
	bind -x '"1":"rosparam set robot_name pr && reset && exit"'  # relancer la simulation du PR (pmi - petit robot)
	bind -x '"2":"rosparam set robot_name gr && reset && exit"'  # relancer la simulation du GR (gros robot)

	bind -x '"0":"rostopic pub --once /game/start std_msgs/Int16 1"'  # envoyer le signal de départ

	bind -x '"4":"rostopic pub --once /game/color std_msgs/Int16 0"'  # côté HOME
	bind -x '"5":"rostopic pub --once /game/color std_msgs/Int16 1"'  # côté AWAY
}
EOF
  echo "Added bindkeys function in bashrc"
  
  else echo "Bindkeys already defined"
fi

echo "Done !"
