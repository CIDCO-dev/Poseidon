#!/bin/bash

# Install utilities with the YES answer to apt prompt
sudo apt install gcc python3-dev python3-pip python3-setuptools -y

# Install RPi.GPIO
pip3 install RPi.GPIO

# Now ROS Installation

# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

# Actual Installation
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base g++ -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

