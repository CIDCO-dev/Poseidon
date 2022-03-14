#!/bin/sh

echo "<*> disable auto-update"
sudo systemctl stop unattended-upgrades

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
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Refresh repos with new ROS repos
sudo apt-get update | tee -a log.txt

#sudo apt install ros-kinetic-ros-base g++ -y >> log.txt
sudo apt install ros-noetic-ros-base ros-noetic-tf2-geometry-msgs g++ python3-rosdep ros-noetic-mavros ros-noetic-sbg-driver -y | tee -a log.txt

echo "[+] Initializing ROS dependencies"
sudo rosdep init | tee -a log.txt
sudo rosdep update | tee -a log.txt

echo "Installing ROS tools"
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y | tee -a log.txt

echo "cleaning gpsd_client"
sudo rm -rd /home/ubuntu/Poseidon/src/workspace/src/gpsd_client
sudo rm -rd /home/ubuntu/Poseidon/src/workspace/src/gps_umd
sudo rm -rd /home/ubuntu/Poseidon/src/workspace/src/gps_common

echo "Downloading GPSD-Client"
cd /home/ubuntu
git clone https://github.com/swri-robotics/gps_umd.git
cd /home/ubuntu/gps_umd
sudo mv gps* /home/ubuntu/Poseidon/src/workspace/src/
cd /home/ubuntu
sudo rm -rd gps_umd

