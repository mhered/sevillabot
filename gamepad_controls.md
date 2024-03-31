## Connect gamepad directly to the robot

We bought the Logitech 710 with a USB dongle. Allows running `/joy_node` from the bot, less lag, no need to carry the laptop around.

## Write a Python node to monitor `/joy` topic and send commands to the Arduino 

I wrote  [joy_subscriber.py](./sevillabot/scripts/joy_subscriber.py)

To get a mapping of your gamepad:

```bash
(T1):$ ros2 run joy joy_node
(T2):$ ros2 topic echo /joy
```

The Logitech 710 has two modes: X and D.  Instructions recommend mode X but the mapping seems strange. Note for example Left joystick only has discrete values in mode X.
Note also that MODE and VIBRATION do not trigger any response in `/joy` topic

|             | Mode X                                                       | Mode D                                                       |
| ----------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| buttons[0]  | A                                                            | X                                                            |
| buttons[1]  | B                                                            | A                                                            |
| buttons[2]  | X                                                            | B                                                            |
| buttons[3]  | Y                                                            | Y                                                            |
| buttons[4]  | LB                                                           | LB                                                           |
| buttons[5]  | RB                                                           | RB                                                           |
| buttons[6]  | BACK                                                         | LT                                                           |
| buttons[7]  | START                                                        | RT                                                           |
| buttons[8]  | ?                                                            | BACK                                                         |
| buttons[9]  | Left Stick Push on                                           | START                                                        |
| buttons[10] | Right Stick Push on                                          | Left Stick Push on                                           |
| buttons[11] |                                                              | Right Stick Push on                                          |
| axes[0]     | Cross Left/Right. Discrete values Left= +1. None = 0, Right = -1 | Left Stick Left/Right. Float values from +1 (Left) to -1 (Right) |
| axes[1]     | Cross Up/Down. Discrete values Up = +1, None = 0, Down = -1  | Left Stick Up/Down. Float values from +1 (Up) to -1 (Down)   |
| axes[2]     | LT. Float values from +1 (Not pressed) to -1 (Fully pressed) | Right Stick Left/Right. Float values from +1 (Left) to -1 (Right) |
| axes[3]     | Right Stick Left/Right. Float values from +1 (Left) to -1 (Right) | Right Stick Up/Down. Float values from +1 (Up) to -1 (Down)  |
| axes[4]     | Right Stick Up/Down. Float values from +1 (Up) to -1 (Down)  | Cross Left/Right. Discrete values Left= +1. None = 0, Right = -1 |
| axes[5]     | RT. Float values from +1 (Not pressed) to -1 (Fully pressed) | Cross Up/Down. Discrete values Up = +1, None = 0, Down = -1  |
| axes[6]     | Left Stick Left/Right. Discrete values Left= +1. None = 0, Right = -1 |                                                              |
| axes[7]     | Left Stick Up/Down. Discrete values Up = +1, None = 0, Down = -1 |                                                              |

## Fix the project to compile the node `joy_subscriber.py`. 

I found the fix here:
https://answers.ros.org/question/299269/ros2-run-python-executable-not-found/

- Add the Python source file to `sevillabot/scripts`
- Add an empty `__init__.py` file in the folder `sevillabot/scripts`
- Make sure the file is executable with `chmod +x joy_subscriber.py`

- Make sure the first line of code is the python shebang:  `#!/usr/bin/env python3`
- In `CMakeLists.txt`:

> ```
> # Install Python executables
> ament_python_install_package(scripts)
> 
> ...
> 
> install(
> 	PROGRAMS scripts/joy_subscriber.py  
>   	DESTINATION lib/${PROJECT_NAME} 
>   )
> ```



Apparently mixing CPP and python in a package is a big deal, see more details here:
https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

Not sure how I got it to work the first time but I am pretty sure I didnt add the `install ( PROGRAMS...` part.

