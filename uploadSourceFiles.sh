#!/bin/bash

rsync -av --delete --filter="merge rsyncFilterList.txt" ./ pi@192.168.1.11:isae-bots-hn-2024/