#!/bin/bash

echo --------------------------------------------------
echo Downloading and installing Poseidon
echo --------------------------------------------------
cd ~/
sudo rm -rd Poseidon
sudo apt install libgps-dev ros-melodic-rosbridge-server
git clone https://github.com/Ddoiron-cidco/Poseidon.git
source /opt/ros/melodic/setup.bash
cd ~/Poseidon/src/workspace/
git clone https://github.com/Ddoiron-cidco/ros_gpsd.git
mv ~/Poseidon/src/workspace/ros_gpsd/gpsd_client ~/Poseidon/src/workspace/src
mv ~/Poseidon/src/workspace/ros_gpsd/gps_common ~/Poseidon/src/workspace/src
mv ~/Poseidon/src/workspace/ros_gpsd/gps_umd ~/Poseidon/src/workspace/src
cd ~/Poseidon/src/workspace/
catkin_init_workspace
cd /home/ubuntu/Poseidon/
catkin_make -j1

echo --------------------------------------------------
echo End
echo --------------------------------------------------
