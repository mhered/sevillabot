#!/bin/bash

# Open a new terminal and run the Joy_Subscriber node on the robot
gnome-terminal --title="Addon (SSH)" -- bash -c "ssh -t mhered@sevillabot 'cd ~/robot_ws; source install/setup.bash; ros2 run sevillabot joy_subscriber.py'
; exec bash"


