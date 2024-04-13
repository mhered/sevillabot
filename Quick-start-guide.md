# Quick Start Guide

## Start sevillabot

### Charge the battery

Balance charging the LiPo battery using the iMAX B6AC LiPo balance charger:

1.  Plug the charger to mains to power it up
2. Check that the jacks of the charging cable are correctly connected to the charger (black to negative and red to positive). 
3.  Connect the XT60 connector of the battery to the charging cable. 
4.  Insert the JST-XH balance connector of the battery to the designated slot in the balance charger (In our case 3 cells). 
5.  Use arrows to go to the **Balance Charge** setting.
6.  Select **LiPo** battery type. 
7. Confirm settings. For the 5000mAh 11.1V 50C LiPo battery: **Lipo**,  **3.8A** charging current (below 1C) and **11.1V (3S)** configuration. 
8.  Hold the **Start** button to start. Press it again to confirm. 
9. During charging: use arrows to move to the screen with 6 voltages. It is normal to see the lower three indications static at 0.00. This is because the charger is capable of charging batteries with up to 6 cells, but our battery actually has only three.
10. The charger alarm will beep when charge is complete

### Switch on the robot

Red switch on the side from **O** to **I**

### Spawn the robot

SSH into the robot , source ROS2 workspace and spawn the robot:

```bash
(PC T1)$ ssh mhered@sevillabot
mhered@192.168.8.170's password: 
...
(bot T1)$ cd ~/robot_ws/
(bot T1)$ source install/setup.bash
(bot T1)$ ros2 launch sevillabot launch_robot.launch.py
```

Note: After the modification made to the launch file, this now spawns the robot, launches `twist_mux` and launches the gamepad launcher `joystick.launch.py` .

### Move the robot

#### With a gamepad Logitech F710 from bot (default)

1. Connect gamepad USB dongle to RPi top left USB port (to free reserved ports to Arduinos)

2. Ensure gamepad mode is set to **X** (check switch in front face of gamepad)

3. **Note: this step is no longer needed, it is included in the robot launch file**. To control only robot the base: ssh into RPi, source and launch joystick:

```bash
(PC T2)$ ssh mhered@sevillabot
mhered@192.168.8.170's password: 
...
(bot T2)$ cd ~/robot_ws/
(bot T2)$ source install/setup.bash
(bot T2)$ ros2 launch sevillabot joystick.launch.py
```



Note: the controls are defined in a parameter file which implements:

- Dead man switches: LB button (left shoulder) for normal speed, RB button (right shoulder) for turbo
- Control on left stick: vertical axis for forward/backward motion and horizontal axis for rotation.



4. to control also addons, ssh again into RPi, source and launch `joy_subscriber` node:

```bash
(PC T3)$ ssh mhered@sevillabot
mhered@192.168.8.170's password: 
...
(bot T3)$ cd ~/robot_ws/
(bot T3)$ source install/setup.bash
(bot T3)$ ros2 launch sevillabot joy_subscriber.py
```

- For zombie challenge: 
  - Y button to toggle laser target
  - X to fire 
  - Digital pad UP / DOWN to adjust aim up or down
- For Eco disaster
  - Digital pad LEFT / RIGHT to open and close the claw

### Binbok wireless from PC

Connect the gamepad to PC in Ubuntu:

* ensure it is charged (or charge with microUSB)
* to pair press HOME + SHARE (small button labelled 'S' on the left above the cross) until light flashes white (note if you press HOME + 'O' button on the right the white light blinks, not flashes)
* In Bluetooth settings select Wireless Controller to Connect. Light stops flashing and turns blue
* You may want to test it works with `$ evtest`

Launch gamepad controller:

```bash
(PC T2)$ ros2 launch sevillabot joystick.launch.py # not anymore needed, included robot launch file 
(PC T3)$ ros2 launch sevillabot joy_subscriber.py
```

#### With keyboard from PC

This is an alternative for base robot only if no gamepad is available. 

Launch keyboard teleop controller in PC:

```bash
(PC T2)$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

Note remapping of topic `/cmd_vel` to `/cmd_vel_joy` (before was `/diff_cont/cmd_vel_unstamped` but this changed after using `twist_mux`)

Now you can move the robot from computer with the keyboard and see it in RVIZ!

Note: keyboard controls for addons are implemented only partially, see `code/command_claw.py` and `code/command_gun.py`

### Launch RVIZ with config file in the PC

Source ROS2 workspace in PC and launch RVIZ:

```bash
(PC T4)$ cd ~/dev_ws/
(PC T4)$ source install/setup.bash
(PC T4)$ rviz2 -d ~/dev_ws/src/sevillabot/config/bot_with_sensors.rviz
```

Note we use a predefined config file but it is not necessary

### Sensors

#### Stream camera to PC

SSH to the robot from another Terminal and launch the camera controller:

```bash
(PC T5)$ ssh mhered@sevillabot
mhered@192.168.8.170's password: 
...
(bot T4)$ cd ~/robot_ws/
(bot T4)$ source install/setup.bash
(bot T4)$ ros2 launch sevillabot camera.launch.py
```

In RVIZ Add Camera and select topic `/image_raw/compressed`

It does not show the camera feed in RVIZ as it should... BUT it works with RQT:

```bash
(PC T6)$ cd ~/dev_ws/
(PC T6)$ source install/setup.bash 
(PC T6)$ ros2 run rqt_image_view rqt_image_view
```

#### Stream Lidar scans

Launch the LIDAR controller:

```bash
(bot T5)$ cd ~/robot_ws/
(bot T5)$ source install/setup.bash
(bot T5)$ ros2 launch sevillabot ld06.launch.py 
```

## Power off sevillabot

```bash
$ sudo shutdown -h now
```

Wait a bit for RPi to power down before switching off mains red button.



**NOT YET IMPLEMENTED**

Safe reset and shutdown: the blue button.

* Shortly pressing the Blue button is ignored (considered noise)
* ~1 sec press resets the RPi (not sure if there is a use case for this)
* 4 sec press shuts down safely the RPi

After Lidar powers off (means RPi powered off) switch off mains red button

See implementation in file `safe_shutdown.py` in: [](/home/mhered/manolobot/code/button/) 

## To Do

- [x] copy code over to sevillabot and update calls
- [ ] ** calibrate for sevillabot robot dimensions
- [ ] ** optimize controls. e.g. move left-right to right joystick?
- [ ] ** integrate `joy_subscriber` in the joystick launch file to control gun and clamp addons ??
- [ ] ** read addon controls from the `joystick.yaml` param file 

- [ ] add photos / video of battery charging
- [ ] fix issue with camera in RVIZ
- [ ] implement power down button making it more usable (remove short press for reset and provide feedback for shutdown)
- [ ] wheels skid, big time
- [ ] motors have a lot of hysteresis