# Raspberry Pi 4B 4Gb setup for ROS 2 Humble

ROS2 Humble runs on Ubuntu 22.04 Jammy Jellyfish 

Note: I follow these instructions: https://articulatedrobotics.xyz/ready-for-ros-1-what-you-need/ which lead here: https://www.techradar.com/how-to/how-to-install-ubuntu-on-the-raspberry-pi then adapt them following these guidelines to port to Humble: https://www.youtube.com/watch?v=qoj5_fVBPII 

1. Buy a 64Gb microSD card as recommended here: https://ubuntu-mate.org/raspberry-pi/. Interesting to check for other RPi specific notes

2. Format the card FAT32. Not straightforward from a Windows11 computer, in which the GUI only allows exFAT for drives >32Gb, may command line: `> format /FS:FAT32 E:` but it's veeery slow! Instead I did it in Ubuntu.

3. Download the Ubuntu MATE 22.04.3 LTS arm64 image from here: https://ubuntu-mate.org/download/arm64/jammy/ You may want to verify the checksum (in Windows use`> certutil -hashFile PATH-TO-FILE SHA256`)

4. Flash the image onto the card using e.g. balena etcher. Windows portable version can be downloaded as exe from here: https://etcher.balena.io/ and run with a double-click, and flashing was reasonably fast, but verification failed, also on a second attempt. I tried from linux and it worked.

5. Insert the SD card in RPi, connect it to a keyboard, mouse, screen and power, switch it on, and follow the steps of the ubuntu MATE OS setup process.
6. update and upgrade. This fixes some bugs with Welcome, Software Boutique and others, see: https://ubuntu-mate.community/t/ubuntu-mate-welcome-and-software-boutique-fail-to-run-with-similar-errors/25371):
```bash
$ sudo apt update
$ sudo apt upgrade
```

7. Install needed software 

   1. git with: ``$ sudo apt install git`

   2. arduino IDE 1 for ARM 64b (Note the latest IDE2 is not compatible with ARM)

   3. ssh

   4. ROS

   5. python3 serial

   6. v4lutils

   7. ros humble v4l2 camera

   8. ros humble rplidar 

8. Apply needed config fixes (see https://www.youtube.com/watch?v=qoj5_fVBPII ) 

   1. add user to video and dialout groups then log out
   2. set up network
   3. fix screen rotation
   4. uninstall braille package which conflicts with `$ sudo apt remove brltty`
   5. reinstate legacy camera driver


   

