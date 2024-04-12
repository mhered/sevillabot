import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node



def generate_launch_description():

    # Check parameter to toggle use of ros2_control
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    package_name='sevillabot'

    # Include the robot_state_publisher launch file

    rsp_launch_file_path = os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )

    # Enabling sim time and passing use_ros2_control parameter        
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rsp_launch_file_path]),
                launch_arguments={
                        'use_sim_time': 'true', 
                        'use_ros2_control': use_ros2_control
                    }.items()
        )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_path = os.path.join(
        get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    gazebo_launch_file_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_launch_file_path]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
    )

    #  spawner node from the gazebo_ros package. 
    # The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', package_name],
            output='screen',
        )

    # include the gamepad launch file

    # YAML file with joystick parameters
    joystick_launch_file_path = os.path.join(
                     get_package_share_directory(package_name),'launch',
                    'joystick.launch.py'
                )
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([joystick_launch_file_path]), 
                launch_arguments={'use_sim_time': 'true'}.items()
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
        parameters=[twist_mux_params_file_path,{'use_sim_time': True}],
        # Remap the cmd_vel_out topic from twist_mux to /diff_cont/cmd_vel_unstamped
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')], 
    )


    # Include the controller manager
    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["diff_cont"],
        )

    # Include the joint state broadcaster
    joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_broad"],
        )

    # Launch them all
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        rsp,
        gazebo,
        spawn_entity,
        joystick,
        twist_mux,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
