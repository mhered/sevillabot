#!/bin/bash


# Open a new terminal and run the image viewer
gnome-terminal --title="Image Viewer" -- bash -c "cd ~/dev_ws;source install/setup.bash;ros2 run rqt_image_view rqt_image_view
; exec bash"