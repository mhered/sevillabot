# /commands/README.md
This folder contains bash commands to be used as shortcuts to interact with sevillabot. 

Open a terminal in this folder and launch commands. 

Each command opens a separate terminal.

## Shortcuts for frequent remote commands in bot

| Usage                   | Commands sent to bot over SSH                                | Comments                                                     |
| ----------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `$ . activate_bot.bash` | `$ cd ~/robot_ws`<br />`$ source install/setup.bash`<br />`$ ros2 launch sevillabot launch_robot.launch.py` | Spawns the robot. Sources ROS2 in the workspace then runs launch file that spawns the robot, launches twist_mux and joystick. |
| `$ . shutdown_bot.bash` | `$ sudo shutdown -h now`                                     | Shuts down the RPi                                           |
| `$ . miniterm.bash`     | `$ miniterm`                                                 | Launches miniterm                                            |
| `$ . camera.bash`       | `$ cd ~/robot_ws`<br />`$ source install/setup.bash`<br />` $ ros2 launch sevillabot camera.launch.py` | Launches camera node                                         |
| `$ . lidar.bash`        | `$ cd ~/robot_ws`<br />`$ source install/setup.bash`<br />` $ ros2 launch sevillabot camera.launch.py` | Launches lidar node                                          |



## Shortcuts for frequent commands in PC


| Usage                 | Commands executed in PC                                      | Comments                                     |
| --------------------- | ------------------------------------------------------------ | -------------------------------------------- |
| `$ . RVIZ.bash`       | `$ rviz2 -d ~/dev_ws/src/sevillabot/config/bot_with_sensors.rviz` | Launches RVIZ with config file to view Lidar |
| `$ . view_image.bash` | `$ cd ~/dev_ws`<br />`$ source install/setup.bash`<br />` $ ros2 launch sevillabot camera.launch.py` | Launches RQT Image viewer                    |



## Sources

Inspired on this article: https://malcontentcomics.com/systemsboy/2006/07/send-remote-commands-via-ssh.html 
Check out this one too if need to send variables: https://malcontentcomics.com/systemsboy/2006/11/using-ssh-to-send-variables-in-scripts.html

Check this article for ways to launch several terminals or tabs with different titles: https://www.baeldung.com/linux/gui-open-terminals-run-commands

Check this article for ways to concatenate commands: https://unix.stackexchange.com/questions/159489/is-there-a-difference-between-and-and
