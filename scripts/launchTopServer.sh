#!/bin/bash

#### script exécuté à l'allumage de la pi ####



# setup de la liaison TCP de pulseaudio sur la pi en-dehors du docker 
# a faire sur la pi au démarrage
pactl load-module module-native-protocol-tcp port=34567


# lancement du docker avec le script python topServer.py
cd /home/pi/isae-bots-hn-2024
make main CMD="cd dev/src/top; python topServer.py"

echo "Exited TopServer"