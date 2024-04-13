#!/bin/bash

# Open a new terminal and run miniterm on the robot
gnome-terminal --title="Camera (SSH)" -- bash -c "ssh -t mhered@sevillabot 'miniterm'
; exec bash"
