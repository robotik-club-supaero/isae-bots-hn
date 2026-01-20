#!/bin/bash

rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@robot.local:isae-bots-hn-2025/
#rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@192.168.1.3:isae-bots-hn-2025/