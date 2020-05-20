#!/bin/bash

echo --------------------------------------------------
echo Downloading and installing Poseidon
echo --------------------------------------------------
cd ~/
sudo rm -rd Poseidon
sudo apt install libgps-dev ros-melodic-rosbridge-server
git clone https://github.com/Ddoiron-cidco/Poseidon.git
source /opt/ros/melodic/setup.bash
cd /home/ubuntu/Poseidon/

echo --------------------------------------------------
echo End
echo --------------------------------------------------
