#!/bin/bash

rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@robot.local:isae-bots-hn/
#rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@192.168.1.3:isae-bots-hn/