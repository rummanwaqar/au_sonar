# BBB Install guide

## Easy Way
* Download the prebuilt [sonar image](https://drive.google.com/a/ualberta.ca/file/d/1wReoiDjcon2ny3Dl7qBWqaYGj9TlAKMU/view?usp=sharing) from Team Drive. 
* Use [etcher](http://etcher.io/) to install image on SDCard
* Login into BBB using debian:temppwd
* Make sure to `git pull` au_sonar repo to get the latest updates. 

## Hard Way 👷
### Write Linux Image 
* Download and install from [beaglebone](https://beagleboard.org/latest-images) of Debian for Beaglebone black (tested kernel: Debian 9.2 2017-10-10 4GB SD IoT). Extract it
```
wget https://debian.beagleboard.org/images/bone-debian-9.2-iot-armhf-2017-10-10-4gb.img.xz
unxz bone-debian-9.2-iot-armhf-2017-10-10-4gb.img.xz
```
* Use [etcher](http://etcher.io/) to install image on SDCard
* Login into BBB using debian:temppwd

### Setting up static IP Address (10.42.43.125)
Edit `/etc/network/interfaces` and add the following 
```
auto eth0
iface eth0 inet static
  address 10.42.43.125
  netmask 255.255.255.0
  gateway 10.42.43.1
```

### Update date (make sure you are connected to the internet)
```
sudo apt update
sudo apt install ntpdate
ntpdate -b -s -u pool.ntp.org
```

### Update kernel and bootloader (only works with 4.4 kernel)
```
sudo /opt/scripts/tools/update_kernel.sh --bone-kernel --lts-4_4
sudo /opt/scripts/tools/developers/update_bootloader.sh
sudo reboot
```

### Modify boot settings (modify _/boot/uEnv.txt_) and enable UIO PRUSS
* Disabled HDMI, Audio and EMMC by uncommentting
```
disable_uboot_overlay_video=1
disable_uboot_overlay_audio=1
disable_uboot_overlay_emmc=1
```
* Enable UIO instead of remote proc by uncommenting `uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo` and comment any other lines under PRUSS Options
* Ensure Universal Cape is enabled (a.k.a uncommented)	
* Set UIO memory size (2097152) by appending `cmdline=` with `cmdline=... uio_pruss.extram_pool_sz=2097152`
* Reboot
* Check bootloader settings `sudo /opt/scripts/tools/version.sh`. If 
* Check UIO memory size `cat /sys/class/uio/uio0/maps/map1/size`

### Install PRU libraries and PASM
```
git clone https://github.com/beagleboard/am335x_pru_package.git
cd am335x_pru_package/pru_sw/app_loader/interface/
make CROSS_COMPILE=""
cd ../
sudo cp lib/* /usr/lib/
sudo cp include/* /usr/include/
cd ~
```
Building PASM
```
cd am335x_pru_package/pru_sw/utils/pasm_source/
./linuxbuild
cd ../
sudo cp pasm /usr/bin
cd ~
```

### Install dependencies
```
sudo apt install cmake libzmq3-dev libmsgpack-dev python-msgpack
sudo pip install pyzmq
sudo pip install -U platformio
```

### Install ROS-Base
```
sudo apt-get install dirmngr
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

# Install ROS from source:
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

sudo rosdep init
rosdep update

mkdir ~/ros_source_build_ws
cd ~/ros_source_build_ws

rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --exclude roslisp --tar > kinetic-ros_comm-wet.rosinstall
wstool init -j8 src kinetic-ros_comm-wet.rosinstall

rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:stretch
```
Plug in a USB flash drive that can be formatted and used as swap space for the ROS build.

It should be 4GB or greater.

Get the device identifier with: `sudo blkid`

Replace `XXX` with the device identifier:
```
sudo mkswap /dev/XXX
sudo swapon /dev/XXX
```

Build ROS (may take a couple hours):
```
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
```

Finally, add this to the end of `~/.bashrc`:
`source /opt/ros/kinetic/setup.sh`

### Downloading and building this repo
```
git clone https://github.com/arvpUofA/au_sonar.git
cd au_sonar/bbb
make clean
make
```
