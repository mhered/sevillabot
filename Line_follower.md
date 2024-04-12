# Line Follower

## Setup

### Custom messages

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

## Quick Start Guide for Gazebo

Plug the line follower add-on

Launch  publisher, PID, gazebo, RVIZ and twist_mux

```bash
(in VSCODE Terminal!!)$ ros2 run sevillabot line_follower_publisher.py 

(PC T1)$ ros2 run sevillabot pid.py 

(PC T2)$ ros2 launch sevillabot launch_sim.launch.py world:=~/dev_ws/src/sevillabot/worlds/obstacles.world

(PC T3)$ rviz2 -d ~/dev_ws/src/sevillabot/config/slam_gazebo.rviz

(PC T4)$ ros2 run twist_mux twist_mux --ros-args --params-file ~/dev_ws/src/sevillabot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

```

When moving the sensor away from the line the robot turns to counter the error!!

## Quick Start Guide Robot

### Launch the robot

Currently requires three calls, each of which requires to ssh into sevillabot, `cd ~/robot_ws`, `source install/setup.bash`. I should combine these three in a single launch file.

```
(bot T1)$ ros2 launch sevillabot launch_robot.launch.py # launch robot

(bot T2)$ ros2 launch sevillabot joystick.launch.py # launch gamepad

(bot T3)$ ros2 run twist_mux twist_mux --ros-args --params-file ~/robot_ws/src/sevillabot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped # launch twist_mux
```

### Launch the subscriber

I don't know yet why the subscriber is not launching properly but found a hack that works:

1. miniterm onto Arduino. Note this triggers calibration (2 solid LED in Arduino) and it takes a little while before it starts publishing data. Move around the sensor in and out the line during the calibration period, otherwise data will not be reliable later. When calibration finishes, LEDs in the Arduino start blinking.

```bash
(bot T4)$ miniterm 

--- Available ports:
---  1: /dev/ttyAMA0         'ttyAMA0'
---  2: /dev/ttyUSB0         'USB2.0-Ser!'
---  3: /dev/ttyUSB1         'USB2.0-Ser!'
--- Enter port index or full name: 3

E:12568	3500 #frozen during calibration
E:12568	3500
E:12568	3500
E:12568	3500
....
E:16367	3500 # then starts transmitting
E:16620	3501
E:16873	3502
E:17127	3498
E:17380	3499
E:17633	3505
```

2. launch the node in another terminal

```bash
(bot T5)$ ros2 run sevillabot line_follower_publisher.py 
```

3. close miniterm in the previous terminal with `CTRL + ]`. This somehow wakes up the node and it starts publishing. 

```bash
(bot T4)$ # exit miniterm with CTRL + ] : hack to get line_follower_publisher.py to publish!!
```

### Launch the PID node

Warning: put the robot on blocks because soon as you launch this node the robot starts moving. 

We can reuse Terminal 4 after closing miniterm

```bash
(PC T4)$ ros2 launch sevillabot pid.launch.py 
```

The advantage of using the launch file instead of running the node directly (`ros2 run sevillabot pid_node.py`) is that the launch file reads the PID gains and linear speed from the YAML file line_follower_pid.yaml. An example that works for speed = 0.1:

```yaml
line_follower_controller:
  ros__parameters:
    Kp: 20.0
    Ki: 0.0
    Kd: 2.0
    linear_vel: 0.1
```

For 0.15:

```yaml
line_follower_controller:
  ros__parameters:
    Kp: 30.0
    Ki: 0.0
    Kd: 3.0
    linear_vel: 0.15
```

You can modify the parameters in the file and relaunch the node without the need for recompiling.

## To do

- [ ] Understand and fix why the `line_follower_publisher.py` node only works when launched from vs_code terminal (??) -> Seems related to serial communication. Opening and closing miniterm seems to work as well.
- [x] modify `pid_node.py` to read PID constants and linear speed from `line_follower_pid.yaml` -> this will simplify fine tuning
- [ ] integrate gamepad and twist_mux in the robot launch file `launch_robot.launch.py` -> this will simplify launching!
- [ ] maybe integrate also line_follower_publisher.py and pid.py in a `launch_follower.launch.py`, this will simplify launching!
