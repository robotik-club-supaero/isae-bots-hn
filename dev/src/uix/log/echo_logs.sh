#!/bin/bash

CLICOLOR_FORCE=1 ros2 topic echo /rosout | sed -En "/name: $1/{n;s/^.....//;p;}"