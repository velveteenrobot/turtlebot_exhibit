#!/bin/bash

# Set the volume to max
amixer -c 0 sset PCM,0 100%
amixer -c 0 sset Master,0 100%

# Then moo.
aplay /opt/ros/electric/stacks/turtlebot_apps/turtlebot_exhibit/src/cow.wav
