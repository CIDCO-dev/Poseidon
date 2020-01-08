#!/bin/bash

#!!! - Before running these scripts, make sure the Raspberry Pi
#!!!   has done all its inital install and reboot(s) after
#!!!   Ubuntu Core Installation.

echo Make sure we are all up to date
snap refresh

echo Install the classic shell (To use apt)
snap install classic --edge --devmode
echo "######################################"
echo "# Part 1 DONE, now onto Part 2       #"
echo "# When the running command is over,  #"
echo "# run Ubuntu_Core_ROS_Setup_Part2.sh #"
echo "######################################"
echo Go into classic shell
sudo classic
echo Install utilities with the YES answer to apt prompt
sudo apt install gcc python3-dev python3-pip python3-setuptools git curl -y

echo Install RPi.GPIO
pip3 install RPi.GPIO

echo Now ROS Installation

# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

# Actual Installation
sudo apt-get update
sudo apt install ros-kinetic-ros-base g++ -y

echo Initialize rosdep
sudo rosdep init
rosdep update

echo Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
