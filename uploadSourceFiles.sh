#!/bin/bash

rsync -av --exclude-from='rsyncExcludeList.txt' dev/src/ pi@robot:isae-bots-hn-2024/dev/src/