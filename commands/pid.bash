#!/bin/bash

# Open a new terminal and run the Lidar launch file on the robot
gnome-terminal --title="PID (SSH)" -- bash -c "ssh -t mhered@sevillabot 'cd ~/robot_ws; source install/setup.bash; ros2 launch sevillabot pid.launch.py'
; exec bash"


