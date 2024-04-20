#!/bin/bash

# Open a new terminal and spawn the robot
gnome-terminal --title="sevillabot (SSH)" -- bash -c "ssh -t mhered@sevillabot 'cd ~/robot_ws; source install/setup.bash; ros2 launch sevillabot launch_robot.launch.py';
exec bash"
