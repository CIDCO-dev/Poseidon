#!/bin/bash

sudo systemctl stop ros

echo "[+] Building Poseidon"

cd /home/ubuntu/Poseidon/src/workspace
source /opt/ros/noetic/setup.bash
catkin_make -j1

sudo systemctl start ros
sudo systemctl status ros
