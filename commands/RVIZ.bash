#!/bin/bash

# Open a new terminal and run RVIZ2 with the configuration file bot_with_sensors.rviz
gnome-terminal --title="RVIZ" -- bash -c "rviz2 -d ~/dev_ws/src/sevillabot/config/bot_with_sensors.rviz
; exec bash"
