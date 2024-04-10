# SLAM

grid SLAM with a 2D LiDAR

mapping: requires global positioning to record environment

localization: using a map to determine position in global coordinate system

sometimes we only have starting point, detection and odometry and we need to do SLAM

2 categories: feature and grid SLAM. In grid we define a grid of cells and determine if they are occupied/unoccupied

navigation: using maps to determine safe trajectory between locations.

## frame conventions

`base_link` is the frame attached to our robot

`odom` represents world origin as seen by odometry. Note odometry integrates wheel velocities, hence it is smooth but has small drift errors that accumulate over time.

`map` represents the true world origin,  and allows to visualize drift errors. `map` to `odom `jumps around but stays generally correct over time 

`base_footprint` frame is shadow of `base_link` on the 2D ground

## `slam_toolbox`

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
$ ros2 launch sevillabot launch_sim.launch.py world:=~/dev_ws/src/sevillabot/worlds/obstacles.world
```

Gazebo does not start!

First issue: Cannot import name 'GazeboRosPaths' error apparently caused by keeping python scripts in /scripts folder. I moved them to my_scripts.

After long troubleshooting and comparing with articubot_one from Josh Newans I found the differences:

* in file `rsp.launch.py`: add a new `sim_mode` parameter in the call made to `robot.urdf.xacro` , make it same value as `use_sim_time`
* in file `robot.urdf.xacro` declare `sim_mode` xacro parameter with `false` as default. Note this file uses `use_ros2_control` to toggle between `ros2_control.xacro` and a `gazebo_control.xacro`, which does not exist!!
* in `ros2_control.xacro` file use `sim_mode` to toggle between DiffDriveArduino (real robot) and GazeboSystem (simulation). Add `gaz_ros2_ctl_use_sim.yaml` parameter file in call to `libgazebo_ros2_control.so`
* Create `gaz_ros2_ctl_use_sim.yaml` parameter file that sets `use_sim_time` to  `true`



# Navigation

Install dependencies:

```bash
$ sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
$ sudo apt install ros-foxy-twist-mux
```

Will need to remap velocities from `/cmd_vel` where nav2 publishes to `/diff_cont/cmd_vel_unstamped` where our controller expects them, but instead we will multiplex velocities using `twist_mux`. This allows to prioritize, block topics on certain circumstances etc

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

Run with:

```bash
$ ros2 run twist_mux twist_mux --ros-args --params-file ~/dev_ws/src/sevillabot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

[INFO] [1712707933.767525498] [twist_mux]: Topic handler 'topics.joystick' subscribed to topic 'cmd_vel_joy': timeout = 0.500000s , priority = 100.
[INFO] [1712707933.767752264] [twist_mux]: Topic handler 'topics.navigation' subscribed to topic 'cmd_vel': timeout = 0.500000s , priority = 10.
```

