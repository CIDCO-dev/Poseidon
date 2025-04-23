#!/bin/sh

echo -e "\e[35m<*> disable auto-update\[0m"
sudo systemctl stop unattended-upgrades

echo -e "\e[35m[+] Updating repositories\[0m"
sudo apt update | tee log.txt

echo -e "\e[35m[+] Updating base system\[0m"
sudo apt dist-upgrade -y | tee -a log.txt

echo -e "\e[35m[+] Updating applications\[0m"
sudo apt upgrade -y | tee -a log.txt

echo -e "\e[35m[+] Installing toolchain\[0m"
sudo apt install gcc python3-dev python3-pip python3-setuptools git curl zip libcurl4-openssl-dev -y | tee -a log.txt

echo -e "\e[35m[+] Installing ROS\[0m"

# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' | tee -a log.txt

# Setup keys
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Refresh repos with new ROS repos
sudo apt-get update | tee -a log.txt

#sudo apt install ros-kinetic-ros-base g++ -y >> log.txt
sudo apt install ros-noetic-ros-base ros-noetic-tf2-geometry-msgs g++ python3-rosdep ros-noetic-mavros ros-noetic-sbg-driver -y | tee -a log.txt

echo -e "\e[35m[+] Initializing ROS dependencies\[0m"
sudo rosdep init | tee -a log.txt
sudo rosdep update | tee -a log.txt

echo -e "\e[35mInstalling ROS tools\[0m"
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y | tee -a log.txt

echo -e "\e[35mDownloading GPSD-Cliente\[0m"
cd /opt
sudo git clone https://github.com/CIDCO-dev/gps_umd.git
cd /opt/gps_umd
sudo mv gps* /opt/Poseidon/src/workspace/src/
cd /opt
sudo rm -rd gps_umd

