#!/bin/sh

ccd /home/ubuntu/Poseidon/src/workspace/src/

echo "Download mavros"
git clone https://github.com/mavlink/mavros.git


echo "Download VLP-16 driver"
sudo apt-get install ros-noetic-velodyne
git clone https://github.com/ros-drivers/velodyne.git
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

echo "Download sbg driver"
git clone https://github.com/SBG-Systems/sbg_ros_driver.git
sudo adduser $USER dialout
