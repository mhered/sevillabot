#!/bin/bash

# Open a new terminal and run the camera launch file on the robot
gnome-terminal --title="Camera (SSH)" -- bash -c "ssh -t mhered@sevillabot 'cd ~/robot_ws; source install/setup.bash; ros2 launch sevillabot camera.launch.py'
; exec bash"