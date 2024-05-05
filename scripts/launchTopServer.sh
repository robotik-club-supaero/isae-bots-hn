#!/bin/bash

#### script exécuté à l'allumage de la pi ####


# setup de la liaison TCP de pulseaudio sur la pi en-dehors du docker 
# a faire sur la pi au démarrage
sleep 5 # wait for pulseaudio server to be ready

export XDG_RUNTIME_DIR=/run/user/1000 # give access to the user 1000 (pi)

pactl load-module module-native-protocol-tcp port=34567 # create the link with tcp server to be able to connect from docker

# lancement du docker avec le script python topServer.py
cd /home/pi/isae-bots-hn-2024
make main INTERACTIVE="" CMD="cd dev/src/top; python topServer.py; bash"

echo "Exited TopServer"