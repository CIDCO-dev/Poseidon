#!/bin/sh

cd /home/ubuntu/Poseidon/src/workspace/src/

echo "Download mavros"
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs ros-noetic-control-toolbox -y


echo "Download VLP-16 driver"
sudo apt-get install ros-noetic-velodyne
git clone https://github.com/ros-drivers/velodyne.git

rosdep update
rosdep install --from-paths velodyne/ --ignore-src --rosdistro noetic -y


echo "Download sbg driver"
sudo apt-get install ros-noetic-sbg-driver

sudo adduser $USER dialout

sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-usb-serial.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A500D8GK",SYMLINK+="imu"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="AC00Y2Y7" SYMLINK+="sonar"
EOF'



cd /home/ubuntu/Poseidon/src/workspace
catkin_make -j1
