#!/bin/bash

rsync -av dev/src/ pi@192.168.1.11:isae-bots-hn-2024/dev/src/
rsync -av --exclude-from='rsyncExcludeList.txt' scripts/ pi@192.168.1.11:isae-bots-hn-2024/scripts/