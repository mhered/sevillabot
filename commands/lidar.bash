#!/bin/bash

# Open a new terminal and run the Lidar launch file on the robot
gnome-terminal --title="Lidar (SSH)" -- bash -c "ssh -t mhered@sevillabot 'cd ~/robot_ws; source install/setup.bash; ros2 launch sevillabot ld06.launch.py'
; exec bash"


