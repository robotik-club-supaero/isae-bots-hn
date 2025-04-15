#!/bin/bash

rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@robot.local:isae-bots-hn-2024/