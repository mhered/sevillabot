# SLAM

## Intro

grid SLAM with a 2D LiDAR

mapping: requires global positioning to record environment

localization: using a map to determine position in global coordinate system

sometimes we only have starting point, detection and odometry and we need to do SLAM

2 categories: feature and grid SLAM. In grid we define a grid of cells and determine if they are occupied/unoccupied

navigation: using maps to determine safe trajectory between locations.

## Frame conventions

`base_link` is the frame attached to our robot

`odom` represents world origin as seen by odometry. Note odometry integrates wheel velocities, hence it is smooth but has small drift errors that accumulate over time.

`map` represents the true world origin,  and allows to visualize drift errors. `map` to `odom `jumps around but stays generally correct over time 

`base_footprint` frame is shadow of `base_link` on the 2D ground

## Install and Setup

1.  add `base _footprint` link to `robot_core.xacro`

```xml
<!-- BASE_FOOTPRINT LINK -->
    <joint name="base_footprint_joint type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    
    <link name="base_footprint">
    </link>
```

2. install `slam_toolbox`

```bash
$ sudo apt install ros-foxy-slam-toolbox
```

We will run SLAM in online (live) asynchronous (may skip some scans but always process the latest) mode

Copy the parameter file to our project:

```bash
$ cp /opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml sevillabot/sevillabot/config/
```

Run with:

```bash
(PC T1)$ ros2 launch sevillabot launch_sim.launch.py world:=~/dev_ws/src/sevillabot/worlds/obstacles.world
```

Gazebo does not start!

First issue: Cannot import name 'GazeboRosPaths' error apparently caused by keeping python scripts in /scripts folder. I moved them to my_scripts.

After long troubleshooting and comparing with articubot_one from Josh Newans I found the differences:

* in file `rsp.launch.py`: add a new `sim_mode` parameter in the call made to `robot.urdf.xacro` , make it same value as `use_sim_time`
* in file `robot.urdf.xacro` declare `sim_mode` xacro parameter with `false` as default. Note this file uses `use_ros2_control` to toggle between `ros2_control.xacro` and a `gazebo_control.xacro`, which does not exist!!
* in `ros2_control.xacro` file use `sim_mode` to toggle between DiffDriveArduino (real robot) and GazeboSystem (simulation). Add `gaz_ros2_ctl_use_sim.yaml` parameter file in call to `libgazebo_ros2_control.so`
* Create `gaz_ros2_ctl_use_sim.yaml` parameter file that sets `use_sim_time` to  `true`

Launch also rviz:

```bash
(PC T2)$ rviz2 -d ~/dev_ws/src/sevillabot/config/bot_with_sensors.rviz
```

Launch teleop (would be better with gamepad if available):

```bash
(PC T3)$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Launch SLAM:

```bash
(PC T2)$ ros2 launch slam_toolbox online_async_launch.py params_file:=./src/sevillabot/config/mapper_params_online_async.yaml use_sim_time:=true
```

In RVIZ:

* add a Map and set the topic to /map

* change the Fixed Frame from odom to map (this to have the map steady)
* save RVIZ config to `slam_gazebo.rviz`

## Quick Start Guide SLAM on PC with keyboard

```bash
(PC T1)$ ros2 launch sevillabot launch_sim.launch.py world:=~/dev_ws/src/sevillabot/worlds/obstacles.world

(PC T2)$ ros2 launch slam_toolbox online_async_launch.py params_file:=./src/sevillabot/config/mapper_params_online_async.yaml use_sim_time:=true

(PC T3)$ rviz2 -d ~/dev_ws/src/sevillabot/config/slam_gazebo.rviz

(PC T4)$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Note: the above is now obsolete due to use of twist_mux



## Creating a map

Close everything an reopen

Make sure config file `mapper_params_online_async.yaml` is ok: set mode to mapping, map file and start commented out: 

```
mode: mapping # localization or mapping depending on the activity
...
# map_file_name: /home/mhered/sevillabot/sevillabot/maps/map_obstacles_serial
...
# map_start_at_dock: true
```

In RVIZ:

* Drive around to build the map
* May help to switch View to TopDownOrthographic

To save the map:

* In RVIZ Add New Panel
* save map for external services eg navigation produces PGM + YAML, serialize to reuse in slam_toolbox produces DATA + POSEGRAPH
* map files saved by default in the workspace `/dev_ws`

## Using a map

Close everything and reopen

Modify config file `mapper_params_online_async.yaml`: set mode to localization, declare map full path without extension and set start at dock 

```
mode: localization # localization or mapping depending on the activity
...
map_file_name: /home/mhered/sevillabot/sevillabot/maps/map_obstacles_serial
...
map_start_at_dock: true
```

## AMCL Adaptive MonteCarlo Localization

### Setup

Install Nav2 (acml is part of nav2)

```bash
$ sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
```

Several steps:

1. Run a map server (from save version because it is external):

```bash
(PC T1)$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/mhered/sevillabot/sevillabot/maps/map_obstacles_save.yaml -p use_sim_time:=true
```

2. Activate it

```bash
(PC T2)$ ros2 run nav2_util lifecycle_bringup map_server
```

3. Relaunch gazebo, RVIZ and slam_toolbox

In RVIZ take care Map is set to Reliable and Transient Local

4. run acml

```bash
$ ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
```

5. activate it

```bash
$ ros2 run nav2_util lifecycle_bringup amcl
```

# Navigation

## Intro

Finding a safe trajectory between an initial and a target pose
We will need:

1. accurate position estimate ie localization from SLAM
2. awareness of obstacles from 1) a static prerrecorded map used for plnning the trajectory + 2) updated live sensor data. Stored in a costmap (avoid high cost, low cost is safe)
3. info about size and dynamics of the robot

## Install

```bash
$ sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
$ sudo apt install ros-foxy-twist-mux
```

## Multiplexing velocities

Will need to remap velocities from `/cmd_vel` where nav2 publishes to `/diff_cont/cmd_vel_unstamped` where our controller expects them, but instead we will use `twist_mux` node to multiplex velocities. This allows to prioritize, block topics on certain circumstances etc

We will set up `twist_mux` to take joystick velocities from `/cmd_vel_joy` and navigation velocities from `/cmd_vel` combine them and publish them to `/diff_cont/cmd_vel_unstamped` 

Modify   `joystick.launch.py` so it remaps velocities to  `/cmd_vel_joy` instead of directly to`/diff_cont/cmd_vel_unstamped` and add new file `twist_mux.yaml`:

```
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: cmd_vel
        timeout: 0.5
        priority: 10
      joystick:
        topic: cmd_vel_joy
        timeout: 0.5
        priority: 100
```

1. Run with:

```bash
$ ros2 run twist_mux twist_mux --ros-args --params-file ~/dev_ws/src/sevillabot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

[INFO] [1712707933.767525498] [twist_mux]: Topic handler 'topics.joystick' subscribed to topic 'cmd_vel_joy': timeout = 0.500000s , priority = 100.
[INFO] [1712707933.767752264] [twist_mux]: Topic handler 'topics.navigation' subscribed to topic 'cmd_vel': timeout = 0.500000s , priority = 10.
```

2. Launch gazebo, RVIZ and slam_toolbox

3. Launch nav2

```bash
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

4. In RVIZ add a new Map, set topic to `/global_costmap/costmap`, set Color Scheme to costmap, turn off the other map.
5. Set a 2D goal pose. It works!!

# Run SLAM & Navigation in the robot

## Install slam_toolbox and nav2 dependencies

```bash
$ sudo apt install ros-foxy-slam-toolbox
$ sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
$ sudo apt install ros-foxy-twist-mux
```

git pull to update the repo in the bot

## Setup Lidar LD06 on robot

1. Clone and symlink the repo:

```bash
(bot):$ cd ~/git
(bot):$ git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
(bot):$ ln -s ~/git/ldlidar_stl_ros2/ ~/robot_ws/src/
```

2. Plug the LD06 and find out the device id (`by-id` because the path `/dev/ttyUSB*` may change):

```bash
(bot):$ ls /dev/serial/by-id
usb-1a86_USB2.0-Ser_-if00-port0
usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Where `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` is the LD06 ( and`usb-1a86_USB2.0-Ser_-if00-port0` is the arduino)

3. Set device permissions:

```bash
(bot):$ sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

4. Copy `ldlidar_stl_ros2/launch/ld06.launch.py` over to sevillabot repo, update`package_name` and  `port_name` then build the package and source:

```bash 
(bot):$ cp ~/robot_ws/src/ldlidar_stl_ros2/launch/ld06.launch.py ~/robot_ws/src/sevillabot/launch/ld06.launch.py
... # edit file
(bot):$ cd ~/robot_ws
(bot):$ colcon build 
(bot):$ source install/setup.bash
```

## Quick Start Guide robot with sensors

Updated `launch_robot.launch.py`and `launch_sim.launch.py` to spawn robot, launch gamepad and twist_mux with a single command

```bash
(robot T1):$ ros2 launch sevillabot launch_robot.launch.py #spawns robot, launches gamepad and twist_mux
```

Launch Lidar:

```bash
(robot T2):$ ros2 launch sevillabot ld06.launch.py
```

View it from PC with:	

```bash
(PC):$ rviz2 -d ~/dev_ws/src/sevillabot/config/bot_with_sensors.rviz
```

