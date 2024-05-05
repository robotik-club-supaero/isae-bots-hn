#!/bin/bash

#### script exécuté à l'allumage de la pi ####


# Setup de la liaison TCP de pulseaudio sur la pi en-dehors du docker 

# give access to the user 1000 (pi)
export XDG_RUNTIME_DIR=/run/user/1000

# command to create the link with tcp server to be able to connect from docker
COMMAND="pactl load-module module-native-protocol-tcp port=34567"

while true; do
    output=$($COMMAND 2>&1)
    exitcode=$?

    # Check the exit status of the command
    if [ $exitcode -eq 0 ]; then
        echo "Command executed successfully."
        break  # Exit the loop if command succeeded
    else

        if [ "$output" == "Failure: Unknown error code" ]; then
            # this error happens when the command is run several times, so just break
            # this case happens when the topServer restarts
            echo "TopServer restart"
            break
        fi
        # otherwise the error is most likely "Connection failure: Connection refused pa_context_connect() failed: Connection refused"

        echo "Waiting for pulseaudio server to be available ..."
        sleep 1
    fi
done


# lancement du docker avec le script python topServer.py
cd /home/pi/isae-bots-hn-2024
make main INTERACTIVE="" CMD="cd dev/src/top; python topServer.py; bash"

echo "Exited TopServer"