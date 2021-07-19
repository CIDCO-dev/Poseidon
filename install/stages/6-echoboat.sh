#!/bin/sh

cd /home/ubuntu/Poseidon/src/workspace/src/

echo "Download mavros"
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y


echo "Download VLP-16 driver"
sudo apt-get install ros-noetic-velodyne
git clone https://github.com/ros-drivers/velodyne.git

rosdep update
rosdep install --from-paths velodyne/ --ignore-src --rosdistro noetic -y


echo "Download sbg driver"
git clone https://github.com/SBG-Systems/sbg_ros_driver.git
sudo adduser $USER dialout
