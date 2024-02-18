# Raspberry Pi 4B 4Gb setup for ROS2 Humble

Note: I follow these instructions: https://articulatedrobotics.xyz/ready-for-ros-1-what-you-need/ which lead here: https://www.techradar.com/how-to/how-to-install-ubuntu-on-the-raspberry-pi then adapt them following these guidelines to port to Humble: https://www.youtube.com/watch?v=qoj5_fVBPII 

## Install ubuntu MATE 22.04 

ROS2 Humble requires Ubuntu 22.04 Jammy Jellyfish (or derivative)

1. Buy a 64Gb microSD card (as recommended here: https://ubuntu-mate.org/raspberry-pi/).

2. Format the card FAT32. Not straightforward from a Windows 11 computer, which will insist you use exFAT formats for drives >32Gb. Command line: `> format /FS:FAT32 E:` seemed to be working but it's veeery slow! Instead I did it in Ubuntu.

3. Download the Ubuntu MATE 22.04.3 LTS arm64 image from here: https://ubuntu-mate.org/download/arm64/jammy/ You may want to verify the checksum (in Windows use`> certutil -hashFile PATH-TO-FILE SHA256`)

4. Flash the image onto the card using e.g. balena etcher. A Windows portable version can be downloaded from here: https://etcher.balena.io/ and run by double-clicking. Flashing was reasonably fast, but verification failed, twice. I tried from linux instead and it worked the first time.

5. Insert the SD card in the RPi, connect it to a keyboard, mouse, screen and power, switch it on, and follow the steps of the ubuntu MATE OS setup process. This takes time. 

6. Set up the network as part of the OS installation. Note I did not use netplan despite [this recommendation](https://articulatedrobotics.xyz/ready-for-ros-2-networking/)

7. Update and upgrade. This fixes some bugs with Welcome, Software Boutique and others, see: https://ubuntu-mate.community/t/ubuntu-mate-welcome-and-software-boutique-fail-to-run-with-similar-errors/25371):
```bash
$ sudo apt update
$ sudo apt upgrade
```
Also veery sloow.

At the end of this some issues remain:
- [ ] still no audio output from HDMI despite I tried several fixes including selecting output, using HDMI0 (from [here](https://forums.raspberrypi.com/viewtopic.php?t=282220), and modifying config files from [here](https://ubuntu-mate.org/raspberry-pi/).

## Install needed software 

1. **git** with: `$ sudo apt install git`

2. **arduino**  1.8.19 for ARM with: `$ sudo apt install arduino`. Then add the user to the `dialout` group with `$ sudo usermod -a -G dialout <username>` then logout (GUI also asks for this, and reminds logout is required). Serial ports were not working due to a conflict with some Braille package, which needs deinstallling with `$ sudo apt remove brltty` cfr. https://www.youtube.com/watch?v=qoj5_fVBPII. Test with **Blink** and **AnalogReadSerial** from **Examples/01. Basics**

3. **ssh** with: `$ sudo apt install openssh-server` get local IP address of bot with `$ ip addr` and write it down. Now we can remotely ssh from laptop with `ssh <bot_username_bot>@<bot_addr> # e.g. mhered@192.168.8.170`

4. **pyserial-miniterm** (previously `miniterm`, note the change of name in 22.04) with: `$ sudo apt install python3-serial`

5. install and configure vs-code ssh and arduino extensions to remotely flash arduino from RPi4 following these instructions:https://www.youtube.com/watch?v=2BJ-iJF04VA

## Install ROS Humble

Follow these instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

1. If needed, set locale to support UTF-8
(in minimal environments (e.g. docker), the locale may be something minimal like `POSIX`)

```bash
$ locale  # check for UTF-8
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ locale  # verify settings
```

2. Setup Sources

Add the ROS 2 apt repo: 1) ensure [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled, 2) add the ROS 2 GPG key with apt, and 3) add the repo to your sources list.

``` bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe #enable ubuntu universe repo

$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg #add gpg key

$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null # add repo to sources 
```
3. Ensure your system is up to date before installing ROS2 packages. Always recommended, but particularly in 22.04  `systemd` and `udev`-related packages must be updated before installing ROS 2 or your system may crash.

```bash
$ sudo apt update && sudo apt upgrade
```
4. Install ROS 2 Desktop Install which includes ROS, RViz, demos, tutorials.

``` bash
$ sudo apt install ros-humble-desktop
```

5. Install colcon

```bash
$ sudo apt install python3-colcon-common-extensions
```

5. Source the ROS2 environment

```bash
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

6. Test communication between a C++ node running on the laptop and a python node on the bot

```bash
(laptop:)$ ros2 run demo_nodes_cpp talker
...
[INFO] [1662750396.990408713] [talker]: Publishing: 'Hello World: 67'
[INFO] [1662750397.990414028] [talker]: Publishing: 'Hello World: 68'
[INFO] [1662750398.990288416] [talker]: Publishing: 'Hello World: 69'
...
```

​    

```bash
(bot:)$ ros2 run demo_nodes_py listener
...
[INFO] [1662750392.965285945] [listener]: I heard: [Hello World: 63]
[INFO] [1662750393.965151014] [listener]: I heard: [Hello World: 64]
[INFO] [1662750394.965403701] [listener]: I heard: [Hello World: 65]
...
```

Next step: upgrade laptop to Humble (see [this Articulated Robotics video](https://www.youtube.com/watch?v=qoj5_fVBPII) and [this Medium article](https://robofoundry.medium.com/notes-on-upgrading-to-ubuntu-22-04-and-ros2-humble-8149804abc91) )

or maybe use docker?: https://betterprogramming.pub/how-to-use-docker-to-run-multiple-ros-distributions-on-the-same-machine-d851b42aed5

## Install other software

1. v4lutils

2. ros humble v4l2 camera

3. ros humble rplidar 

## Other needed config fixes

See https://www.youtube.com/watch?v=qoj5_fVBPII

1. add user to video group then log out
2. fix screen rotation problem
4. reinstate legacy camera driver

