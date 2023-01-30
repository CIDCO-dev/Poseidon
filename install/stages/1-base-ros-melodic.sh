#!/bin/sh

echo "[+] Updating repositories"
sudo apt update | tee log.txt

echo "[+] Updating base system"
sudo apt dist-upgrade -y | tee -a log.txt

echo "[+] Updating applications"
sudo apt upgrade -y | tee -a log.txt

echo "[+] Installing toolchain"
sudo apt install gcc python3-dev python3-pip python3-setuptools git curl zip -y | tee -a log.txt

echo "[+] Installing ROS"

# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' | tee -a log.txt

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | tee -a log.txt
sudo curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add - | tee -a log.txt

# Refresh repos with new ROS repos
sudo apt-get update | tee -a log.txt

#sudo apt install ros-kinetic-ros-base g++ -y >> log.txt
sudo apt install ros-melodic-ros-base ros-melodic-tf2-geometry-msgs g++ ros-melodic-mavros -y | tee -a log.txt

echo "[+] Initializing ROS dependencies"
rosdep init | tee -a log.txt
rosdep update | tee -a log.txt

echo "Installing ROS tools"
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y | tee -a log.txt
