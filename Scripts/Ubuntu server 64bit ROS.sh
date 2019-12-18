#!/bin/bash

#update the apps
echo --------------------
echo Updating Deb.
echo --------------------
sudo apt update >> log.txt 2> /dev/null

#upgrade the apps
echo --------------------
echo Updating Apps.
echo --------------------
sudo apt upgrade -y >> log.txt 2> /dev/null

# Install utilities with the YES answer to apt prompt
echo --------------------
echo Installing Utilities.
echo --------------------
sudo apt install gcc python3-dev python3-pip python3-setuptools git curl -y >> log.txt 2> /dev/null

# Install RPi.GPIO
pip3 install RPi.GPIO >> log.txt 2> /dev/null
# Now ROS Installation
echo --------------------
echo Installing ROS.
echo --------------------
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' >> log.txt 2> /dev/null
# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 >> log.txt 2> /dev/null
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add - >> log.txt 2> /dev/null

# Actual Installation
sudo apt-get update >> log.txt 2> /dev/null

#sudo apt install ros-kinetic-ros-base g++ -y >> log.txt
sudo apt install ros-melodic-ros-base g++ -y >> log.txt 2> /dev/null

# Initialize rosdep
echo --------------------
echo Initialize ROSdep
echo --------------------
sudo rosdep init >> log.txt 2> /dev/null
echo --------------------
echo Updating ROSdep
echo --------------------
rosdep update >> log.txt 2> /dev/null

echo --------------------
echo Installing Tools
echo --------------------
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y >> log.txt 2> /dev/null

echo --------------------
echo Installing Network-Manager
echo --------------------
sudo apt-get install network-manager -y > log.txt 2> /dev/null
echo --------------------
echo Installing Network Service
echo --------------------
sudo systemctl start NetworkManager.service > log.txt 2> /dev/null
sudo systemctl enable NetworkManager.service > log.txt 2> /dev/null
echo --------------------
echo Creating Wifi HotSpot
echo --------------------
nmcli dev wifi hotspot ifname wlan0 ssid Hydro-B password "cidco1234" > log.txt 2> /dev/null
echo --------------------
echo Installing web server
echo --------------------
sudo apt install lighttpd -y > log.txt 2> /dev/null
echo --------------------
echo Installing web service
echo --------------------
sudo systemctl start lighttpd.service > log.txt 2> /dev/null
sudo systemctl enable lighttpd.service > log.txt 2> /dev/null

clear
#instaling system installation script here
echo --------------------
echo End of script 
echo --------------------
