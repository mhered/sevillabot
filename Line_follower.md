# Custom messages

1. Create a /msg folder and declare custom message TimestampedData.msg

```
int64 timestamp
int32 data
```

2. modify CMakeLists.txt:

```python
...
# find dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare ROS2 interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TimestampedData.msg
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
...
```

3. modify package.xml

```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  ...
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

```

4. compile, source and check it is available

```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 interface list | grep sevillabot
    sevillabot/msg/TimestampedData
```



Note: For some reason when the node is called from normal terminal it freezes and publishes always the same value:

```bash
$ ros2 run sevillabot line_follower_publisher.py
$ # or
$ python3 ./src/sevillabot/my_scripts/line_follower_publisher.py
```

I need to call it from vs_code terminal, and after that it runs well... why? 



## Testing Line Follower

### Quick Start Guide

Plug the line follower add-on

Launch  publisher, PID, gazebo, RVIZ and twist_mux

```bash
(VSCODE Terminal!!)$ ros2 run sevillabot line_follower_publisher.py 

(PC T1)$ ros2 run sevillabot pid.py 

(PC T2)$ ros2 launch sevillabot launch_sim.launch.py world:=~/dev_ws/src/sevillabot/worlds/obstacles.world

(PC T3)$ rviz2 -d ~/dev_ws/src/sevillabot/config/slam_gazebo.rviz

(PC T4)$ ros2 run twist_mux twist_mux --ros-args --params-file ~/dev_ws/src/sevillabot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

```

When moving the sensor away from the line the robot turns
