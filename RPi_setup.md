# Raspberry Pi 4B 4Gb setup for ROS2 Humble

Note: I follow these instructions: https://articulatedrobotics.xyz/ready-for-ros-1-what-you-need/ which lead here: https://www.techradar.com/how-to/how-to-install-ubuntu-on-the-raspberry-pi then adapt them following these guidelines to port to Humble: https://www.youtube.com/watch?v=qoj5_fVBPII 

## Install ubuntu MATE 22.04 

ROS2 Humble requires Ubuntu 22.04 Jammy Jellyfish (or derivative)

1. Buy a 64Gb microSD card (as recommended here: https://ubuntu-mate.org/raspberry-pi/).

2. Format the card FAT32. Not straightforward from a Windows 11 computer, which will insist you use exFAT formats for drives >32Gb. Command line: `> format /FS:FAT32 E:` seemed to be working but it's veeery slow! Instead I did it in Ubuntu.

3. Download the Ubuntu MATE 22.04.3 LTS arm64 image from here: https://ubuntu-mate.org/download/arm64/jammy/ You may want to verify the checksum (in Windows use`> certutil -hashFile PATH-TO-FILE SHA256`)

4. Flash the image onto the card using e.g. balena etcher. A Windows portable version can be downloaded from here: https://etcher.balena.io/ and run by double-clicking. Flashing was reasonably fast, but verification failed, twice. I tried from linux instead and it worked the first time.

5. Insert the SD card in the RPi, connect it to a keyboard, mouse, screen and power, switch it on, and follow the steps of the ubuntu MATE OS setup process. This takes time.

6. Update and upgrade. This fixes some bugs with Welcome, Software Boutique and others, see: https://ubuntu-mate.community/t/ubuntu-mate-welcome-and-software-boutique-fail-to-run-with-similar-errors/25371):
```bash
$ sudo apt update
$ sudo apt upgrade
```
Also veery sloow.

At the end of this some issues remain:
- [ ] still no audio output from HDMI despite I tried several fixes including selecting output, using HDMI0 (from [here](https://forums.raspberrypi.com/viewtopic.php?t=282220), and modifying config files from [here](https://ubuntu-mate.org/raspberry-pi/).

## Install needed software 

1. git with: `$ sudo apt install git`

2. arduino 1.8.19 for ARM with: `$ sudo apt install arduino`. Then add the user to the `dialout` group with `$ sudo usermod -a -G dialout <username>` then logout (GUI also asks for this, and reminds logout is required). Serial ports were not working due to a conflict with some Braille package, which needs deinstallling with `$ sudo apt remove brltty` cfr. https://www.youtube.com/watch?v=qoj5_fVBPII. Test with **Blink** and **AnalogReadSerial** from **Examples/01. Basics**

3. ssh with: `$ sudo apt install openssh-server` get local IP address of bot with `$ ip addr` and write it down. Now we can remotely ssh from laptop with `ssh <bot_username_bot>@<bot_addr> # e.g. mhered@192.168.8.170`

4. ROS

5. python3 serial

6. v4lutils

7. ros humble v4l2 camera

8. ros humble rplidar 

## Other needed config fixes

See https://www.youtube.com/watch?v=qoj5_fVBPII

1. add user to video group then log out
2. set up network
3. fix screen rotation problem
4. reinstate legacy camera driver


