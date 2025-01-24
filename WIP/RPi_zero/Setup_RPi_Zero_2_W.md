# Setup RPi Zero

Followed this tutorial: https://robofoundry.medium.com/running-ros2-foxy-on-raspberry-pi-zero-2w-a4720334d3bb

1. Download [Raspberry Pi Imager for Windows](https://downloads.raspberrypi.org/imager/imager_latest.exe) to write the correct image of Ubuntu 20.04 Focal in a 32Gb SD Card. 
   In the current version there are options to select: 

   * Device: **Raspberry Pi Zero 2 W**
   * OS: **Ubuntu Server 20.04.5 LTS (64-bit)**
   * SD card

   Then you are prompted to apply OS customization settings:

   * in the **General** tab edit hostname, user/pwd, Wifi SSID/pwd, and regional preferences (LAN country, timezone and keyboard)
   * in the **Services** tab activate SSH with password authentication

   With this,there is no need to create/edit ``network-config``. `config.txt` or `bcm2710-rpi-zero-2.dtb` as suggested by the tutorial

2. After burning the image, insert the SD in the Pi Zero, power it up and wait for ~5 minutes to allow for the first boot to complete. If you can connect a monitor wait until text output eventually stops (watch out as there are loong pauses before it finally stops for good). I never got a login prompt, though.
3. Restart the Pi, monitor from the router interface (https://192.168.8.1/ ) until the zero shows up as connected and note the IP address, in my case 192.168.8.182
4. Then ssh into it with: 

```bash
ssh mhered@192.168.8.182
```

5. We can assign a permanent alias to simplify the ssh command to`$ ssh zero` simply adding an entry to `~/.ssh/config` in the PC:

```bash
  Host zero
    HostName 192.168.8.182
    User mhered
```

6. Before running updates, create a swap file - Pi Zero's memory is limited:

```bash
## Please run all commands below without copying $ in front, that's just there to indicate it is a command line and description is in comments starting with ##
## You can check if you have any swap enabled by executing 
free -m
## Create a file:
sudo fallocate -l 1G /var/swapfile
## initialize with zeros (slow)
sudo dd if=/dev/zero of=/var/swapfile bs=1024 count=1048576
## Modify permissions for swapfile
sudo chmod 0600 /var/swapfile
## Create a swap file system on it
sudo mkswap /var/swapfile
## Enable swap
sudo swapon /var/swapfile
## Run following commands to ensure that you indeed have some swap space
free -m
sudo swapon --show
## make it permanent through restarts by adding an entry to /etc/fstab file
sudo nano /etc/fstab
## append following line at the end of /etc/fstab file in nano editor
/var/swapfile swap swap defaults 0 0
## that's it, reboot and make sure to run free -m and sudo swapon --show to make sure the swap file is still maintained after reboot, if it has not, check the entry in /etc/fstab file to make sure it points to exact location of your swap file e.g. /var/swapfile
sudo reboot
```

7. Switch `initramfs` to use ``lz4`` compression - this results in larger output on the boot partition, but is much faster and less memory intensive  iaw Canonical - and update `initramfs`

```bash
## install lz4
sudo apt install lz4
## configure initramfs to use lz4
sudo sed -i -e 's/COMPRESS=zstd/COMPRESS=lz4/' /etc/initramfs-tools/initramfs.conf

## update initramfs boot files (veery slow)
sudo update-initramfs -u

sudo reboot
```

8. update and upgrade (takes a loong time):

```bash
sudo apt update
sudo apt upgrade
```

9. Install ROS2 foxy desktop and dev tools and ROS tools
   Set locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8 # sloow
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```


​	Set up resources: enable the universe repo, add the ROS2 GPG key and add the repo

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

​	Install ROS2 desktop (with barebone I could not get the sample nodes to work) and dev tools

```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-foxy-desktop python3-argcomplete

sudo apt install ros-dev-tools
```

Add sourcing to shell startup script:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

Test ROS2 with basic talker and listener nodes:

```bash
ros2 run demo_nodes_cpp talker
[INFO] [1716508734.685711158] [talker]: Publishing: "Hello World: 0"
[INFO] [1716508735.687527316] [talker]: Publishing: "Hello World: 1"
[INFO] [1716508736.688247732] [talker]: Publishing: "Hello World: 2"
...
```

```bash
ros2 run demo_nodes_py listener
[INFO] [1716508736.689810760] [listener]: I heard: [Hello World: 2]
[INFO] [1716508737.689275068] [listener]: I heard: [Hello World: 3]
[INFO] [1716508738.689336506] [listener]: I heard: [Hello World: 4]
...
```

