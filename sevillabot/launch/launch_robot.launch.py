import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


from launch_ros.actions import Node



def generate_launch_description():


    package_name='sevillabot'

    rsp_launch_file_path = os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )

    # Include the robot_state_publisher launch file 
    # Disabling use of sim time and enabling ros2_control
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rsp_launch_file_path]), 
                launch_arguments={
                        'use_sim_time': 'false', 
                        'use_ros2_control': 'true'
                    }.items()
    )

    # include the gamepad launch file

    # YAML file with joystick parameters
    joystick_launch_file_path = os.path.join(
                     get_package_share_directory(package_name),'launch',
                    'joystick.launch.py'
                )
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([joystick_launch_file_path]), 
                launch_arguments={'use_sim_time': False}.items()
    )

    # Include twist_mux launch file

    # YAML file with twist_mux parameters
    twist_mux_params_file_path = os.path.join(
            get_package_share_directory(package_name),'config','twist_mux.yaml'
        )
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file_path],
        # Remap the cmd_vel_out topic from twist_mux to /diff_cont/cmd_vel_unstamped
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')], 
    )

    # Include the controller manager
    
    # Read robot description from URDF to obtain hardware interfaces
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # YAML file with controllers
    controller_params_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file_path],
    )

    delayed_controller_manager = TimerAction(
        period=3.0, 
        actions=[controller_manager]
        )
    
    # Include the differential drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Include the joint state broadcaster

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])
