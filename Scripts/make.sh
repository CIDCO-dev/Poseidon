#!/bin/bash

echo --------------------------------------------------
echo Making Poseidon
echo --------------------------------------------------
cd ~/
source /opt/ros/melodic/setup.bash
cd /home/ubuntu/Poseidon/
catkin_make -j1

echo --------------------------------------------------
echo End
echo --------------------------------------------------
