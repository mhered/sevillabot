# Raspberry Pi 4B 4Gb setup for ROS2 Foxy

## Install ubuntu MATE 20.04 

ROS2 Foxy requires Ubuntu 20.04 (or derivative)

1. Buy a 64Gb microSD card (as recommended here: https://ubuntu-mate.org/raspberry-pi/).
2. Format the SD card FAT32, e.g. using `gparted` in the ubuntu GUI

### First few failed attempts

1. Download the Ubuntu MATE 20.04.01 arm64 image: `ubuntu-mate-20.04.1-desktop-arm64+raspi.img.xz` from here: https://releases.ubuntu-mate.org/20.04/arm64/ 

2. You may want to verify the checksum of the download with `$ sha256sum ubuntu-mate-20.04.1-desktop-arm64+raspi.img.xz` (should match the content of `ubuntu-mate-20.04.1-desktop-arm64+raspi.img.xz.sha256`)

3. Flash the image onto the card using e.g. https://etcher.balena.io/. This is reasonably fast.

4. Insert the SD card in the RPi, connect it to a keyboard, mouse, screen and power, switch it on, and follow the steps of the Ubuntu MATE OS setup process. 

   Initially boot did not start and complained that `start_x.elf` was not compatible and needed newer software. Turns out RPi 4 apparently required a firmware update (Not sure why, the board is dated 2018...??).

   To fix it: copy manually all files `start*.elf` and `fixup*.dat` from this repo https://github.com/raspberrypi/firmware/tree/master/boot to the boot partition in the SD, following this PDF guide: https://pip.raspberrypi.com/categories/685/documents/RP-003476-WP/RP-003476-WP-1.pdf to update the firmware which I found in this forum: https://forums.raspberrypi.com/viewtopic.php?t=346302

   This allowed me to progress a bit but OS installation crashed during keyboard setup. I tried many things over several days, but eventually I gave up and started again from scratch.

   ### The good way to install ubuntu MATE 20.04 on RPi 4B

   According to this: https://askubuntu.com/questions/1348560/is-ubuntu-20-04-desktop-for-raspberry-pi-4-no-longer-available the proper way to install Ubuntu 20.04 desktop is installing ubuntu 20.04 server first then installing the desktop on top.

   1. Install ubuntu 20.04 server following the official instructions: https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview. This uses `rpi-imager` and does not require formatting the SD.
   2. Customize installation (username, wifi etc) and during first boot **pay attention to wait long enough for `cloud-init` to finish ** before login (it stops thinking for a couple of minutes then spits more text), see this note: https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server

   3. update and upgrade

   ```bash
   $ sudo apt update
   $ sudo apt upgrade
   ```

   I also struggled quite a bit with the upgrade, I got a message similar to this one: https://phoenixnap.com/kb/fix-could-not-get-lock-error-ubuntu but linked to `unattended-upgrade`. Finally fixed it after a rather chaotic series of `reboot`, `kill PID` and `sudo rm /var/lib/apt/lists/lock` commands, no idea which one worked.

   4. Then install Mate on top of ubuntu server: https://gcore.com/learning/how-to-install-mate-on-ubuntu/ (see here other ubuntu flavors for pi: https://waldorf.waveform.org.uk/2020/ubuntu-desktops-on-the-pi.html)

   ```bash
   $ sudo apt install ubuntu-mate-desktop
   ```

   When prompted select `lightdm` display manager

   It is veery sloow.

   Reboot

   It works!

   As proposed  [here](https://ubuntu-mate.org/raspberry-pi/) I installed `sshguard` and modified the sound config. I succeeded to get HDMI audio output from HDMI0 (see [here](https://forums.raspberrypi.com/viewtopic.php?t=282220))

## Install needed software 

1. **git** with: `$ sudo apt install git`
2. **arduino**  1.8.19 for ARM with: `$ sudo apt install arduino`. If needed add the user to the `dialout` group with `$ sudo usermod -a -G dialout <username>` and logout (GUI also asks for this, and reminds logout is required). Test with **Blink** from **Examples/01. Basics**
3. **ssh** with: `$ sudo apt install openssh-server` get local IP address of bot with `$ ip addr` and write it down. Now we can remotely ssh from laptop with `ssh <bot_username_bot>@<bot_addr> # e.g. mhered@192.168.8.170`

`$ ssh mhered@sevillabot` didn't work initially because the laptop remembered the old ECDSA fingerprint (before the downgrade from 22.04), so I deleted it with `ssh-keygen -f "/home/mhered/.ssh/known_hosts" -R "192.168.8.170"`

4. **miniterm** with: `$ sudo apt install python3-serial`

5. install and configure vs-code **ssh and arduino extensions** to remotely flash arduino from RPi4 following these instructions: https://www.youtube.com/watch?v=2BJ-iJF04VA:

   * install ssh extension then connect to sevillabot and it installs a vscode server in the Rpi. Then click on arduino extension and it installs remotely in the RPi.

   * In **Extensions Settings** go to the **Remote** tab, **Arduino path**, find the path from terminal with 

     ```
     $ ls -l `which arduino`
     ```

     Then paste the path removing the file name, e.g. `/usr/bin/arduino` > `/usr/bin/`)

- [ ] PENDING Not finished. Need to have arduino files and I am tired today.

## Install ROS Foxy

Follow the official instructions: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

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
3. Ensure your system is up to date before installing ROS2 packages. 

```bash
$ sudo apt update && sudo apt upgrade
```
4. Install ROS 2 Desktop Install which includes ROS, RViz, demos, tutorials.

``` bash
$ sudo apt install ros-foxy-desktop python3-argcomplete
```

5. Install dependencies

```bash
$ sudo apt install python3-colcon-common-extensions ros-foxy-ros2-control ros-foxy-xacro ros-foxy-ros2-controllers
```

6. Source the ROS2 environment

```bash
$ echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

7. Test communication between a C++ node running on the PC and a python node on the bot

```bash
$ ros2 run demo_nodes_py listener
...
[INFO] [1662750392.965285945] [listener]: I heard: [Hello World: 63]
[INFO] [1662750393.965151014] [listener]: I heard: [Hello World: 64]
[INFO] [1662750394.965403701] [listener]: I heard: [Hello World: 65]
...
```



```bash
(PC):$ ros2 run demo_nodes_cpp talker
...
[INFO] [1662750396.990408713] [talker]: Publishing: 'Hello World: 67'
[INFO] [1662750397.990414028] [talker]: Publishing: 'Hello World: 68'
[INFO] [1662750398.990288416] [talker]: Publishing: 'Hello World: 69'
...
```

Hurray! 

Next steps: 

## Create a backup copy of a MicroSD

See: https://www.youtube.com/watch?v=xSxNJSkSgpk. This is critical to keep a clean image for 20.04/foxy and avoid having to repeat this nightmare.

Plug origin and destination microSD in laptop

```bash
$ sudo fdisk -l # display all detected drives, identify origin and destination drives
$ df -h # check there is enough space in destination drive
$ sudo dd bs=4M if=/dev/sdb of=/home/mhered/sevillabot/SD_images/clean_20.04_with_ROS_foxy.img #make image from input drive and save to output file
(Terminal 2):$ watch ls -latrh sevillabot/SD_images/clean*.img #to monitor progress (otherwise output is not verbose). Expect a long time!
$ sudo dd bs=4M if=/home/mhered/sevillabot/SD_images/clean_20.04_with_ROS_foxy.img of=/dev/sdb #save image to destination SD card
```

Open topics:

- [ ] Why do I have a large unallocated partition?
- [ ] Alternative: Try also using a smaller SD card

## Gamepad setup

```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install joystick jstest-gtk evtest
$ sudo apt-get install libcanberra-gtk-module libcanberra-gtk0
```

Initially unable to pair the controller to the headless RPi following these instructions:  https://salamwaddah.com/blog/connecting-ps4-controller-to-raspberry-pi-via-bluetooth. or these instructions https://simpleit.rocks/linux/shell/connect-to-bluetooth-from-cli/

I got a message that no controller was available, and one of the suggested fixes: `systemctl start hciuart`, failed.

Tried several things, not sure which one worked:

* reinstalled packages: `$ sudo apt install --reinstall bluez pi-bluetooth`
* rebooted
* manually started `hciuart` with `$ systemctl start hciuart`
* rebooted again?

And then the pairing worked: 

```bash
$ sudo bluetoothctl
Agent registered
[CHG] Controller D8:3A:DD:64:AA:36 Pairable: yes
[bluetooth]# agent on
Agent is already registered
[bluetooth]# discoverable on
Changing discoverable on succeeded
[CHG] Controller D8:3A:DD:64:AA:36 Discoverable: yes
[bluetooth]# pairable on
Changing pairable on succeeded
[bluetooth]# default-agent
Default agent request successful
[bluetooth]# scan on
Discovery started
...
[NEW] Device A5:15:66:C1:A9:A9 Wireless Controller #when gamepad started pairing
...
[bluetooth]# connect A5:15:66:C1:A9:A9
Attempting to connect to A5:15:66:C1:A9:A9
[CHG] Device A5:15:66:C1:A9:A9 Connected: yes
...
[CHG] Device A5:15:66:C1:A9:A9 UUIDs: 00001124-0000-1000-8000-00805f9b34fb
[CHG] Device A5:15:66:C1:A9:A9 UUIDs: 00001200-0000-1000-8000-00805f9b34fb
[CHG] Device A5:15:66:C1:A9:A9 ServicesResolved: yes
...
[CHG] Device A5:15:66:C1:A9:A9 Paired: yes
Connection successful
...
[Wireless Controller]# trust A5:15:66:C1:A9:A9
[CHG] Device A5:15:66:C1:A9:A9 Trusted: yes
Changing A5:15:66:C1:A9:A9 trust succeeded
...
[Wireless Controller]# quit
```

I can pair the gamepad to the PC and to the RPi using this procedure, it works on PC with `evtest` but not on RPI4.

Actually the blue light is not so blue when connecting to the Pi...

I can survive with the gamepad connected to the PC for the moment, but like to connect to the bot for speed and reliability.

To do:

- [ ] try pairing the gamepad in the GUI (not practical, but useful for troubleshooting purposes)
- [ ] Josh recommends wireless with dongle as it is easier to pair than bluetooth... Just ordered it.

## Moving the robot

Continue following instructions here: https://github.com/mhered/manolobot/blob/main/Part-5-Teleoperation-Feb23.md

Create a workspace, clone repos and build

Clone robot repo from sevillabot , diffdrive and serial library from josh fork (using "the good way", making a symlink, see https://github.com/mhered/manolobot/blob/main/Part-4-Simulation-Jan23.md):

```bash
$ git clone https://github.com/mhered/sevillabot.git # clone sevillabot
$ cd ~/git
$ git clone https://github.com/joshnewans/serial_motor_demo.git # clone demo node
$ git clone https://github.com/joshnewans/diffdrive_arduino # clone diffdrive
$ git clone http://github.com/joshnewans/serial # clone serial
$ mkdir -p ~/robot_ws/src # create workspace and make symlinks
$ ln -s ~/sevillabot/sevillabot/ ~/robot_ws/src/ 
$ ln -s ~/git/serial_motor_demo/ ~/robot_ws/src/
$ ln -s ~/git/diffdrive_arduino/ ~/robot_ws/src/ 
$ ln -s ~/git/serial/ ~/robot_ws/src 
```

For demo node, build and source workspace, run the node that listens for commands:

```bash
$ cd ~/robot_ws
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 run serial_motor_demo driver --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=57600 -p loop_rate:=30 -p encoder_cpr:=1975
```

In the PC build workspace and run GUI:

```bash
(PC):$ cd dev_ws
(PC):$ source install/setup.bash
(PC):$ $ ros2 run serial_motor_demo gui
```

Hurray! Can control motor over ROS2 with PC and bot in foxy!!

And then, following the instructions in [Quick Start Guide](./Quick-start-guide.md) you can control the robot with a gamepad.

## Camera

1. Connect the Pi camera. Note if camera was connected while the RPi was on, you may need to reboot.

2. Install sfw in Rpi ( **v4l2-utils** video driver library and ros node):

   ```bash
   $ sudo apt update
   $ sudo apt upgrade
   $ sudo apt install libraspberrypi-bin v4l-utils ros-foxy-v4l2-camera
   ```
   
3. If needed add the user to the video group:

   ```bash
   $ sudo usermod -aG video mhered
   ```

   Reboot for group changes to take effect

   ```bash
   $ groups
   mhered adm dialout cdrom sudo audio video plugdev games users input render netdev bluetooth gpio spi i2c
   ```

5. Check the camera is detected and supported with:

```bash
$ vcgencmd get_camera
supported=1 detected=1
```

Initially I got `VCHI initialization failed`. I fixed after adding the line `start_x=1` to file `/boot/firmware/config.txt`, saving and rebooting (solution here: https://askubuntu.com/questions/1211805/raspberry-picamera-on-ubuntu-not-found?rq=1). Well, in actual fact, for 22.04 Josh Newan recommended reinstating the legacy camera driver from 20.04  by editing the following lines in the config file with `(Rpi):$ sudo nano /boot/firmware/config.txt`:

```
...
camera_auto_detect=0 # was 1, modified by MH to reinstate legacy camera driver
display_auto_detect=1
start_x=1 # added by MH to reinstate legacy camera driver
...
```

Check if `v4l` (video for linux) can see the camera with:

```bash
$ v4l2-ctl --list-devices
bcm2835-codec-decode (platform:bcm2835-codec):
	/dev/video10
	/dev/video11
	/dev/video12
	/dev/media1

bcm2835-isp (platform:bcm2835-isp):
	/dev/video13
	/dev/video14
	/dev/video15
	/dev/video16
	/dev/media0

mmal service 16.1 (platform:bcm2835-v4l2):
	/dev/video0
```

Install **image transport library** and **rqt_image_view**:

```bash
$ sudo apt install ros-foxy-image-transport-plugins ros-foxy-rqt-image-view
```

Launch driver node:

```bash
$ ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical
```

Then launch `rqt_image_view` from the PC:

```bash
(PC):$ ros2 run rqt_image_view rqt_image_view
```

Hurray!!  the bot broadcasts video to the laptop.

Note: it broadcasts only over `/image_raw_compressed/`

## Other software and needed config fixes

See https://www.youtube.com/watch?v=qoj5_fVBPII

1. fix screen rotation problem?
2. Setup lidar? ros foxy rplidar 

