from launch import LaunchDescription
from launch_ros.actions import Node

import sys

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Define the package name
    package_name='sevillabot'

    # Build the path to the YAML configuration file
    pid_params_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'line_follower_pid.yaml'
        )

    # Check if the YAML file exists
    if not os.path.exists(pid_params_path):
        sys.stdout.write(f"WARNING: Parameter file '{pid_params_path}' not found, using Default values for PID gains.\n")

    # Configure the node
    pid_node = Node(
            package=package_name,
            executable='pid_node.py',
            name='line_follower_controller',
            parameters=[pid_params_path]
        )

    return LaunchDescription([
        pid_node,
    ])

