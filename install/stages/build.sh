#!/bin/bash

sudo systemctl stop ros

echo "[+] Creating temporary swap file"
FILE=/swap.img
if [ -f "$FILE" ]; then
    echo "$FILE exists."
else 
    echo "$FILE does not exist."
    sudo swapoff -a
    sudo dd if=/dev/zero of=/swaprpi.img bs=1024k count=516
    sudo mkswap /swaprpi.img
    sudo swapon /swaprpi.img
fi

echo "[+] Building Poseidon"

cd /home/ubuntu/Poseidon/src/workspace
source /opt/ros/melodic/setup.bash
catkin_make clean
catkin_make -j1

echo "[+] Disabling temporary swap"
FILE=/swaprpi.img
if [ -f "$FILE" ]; then
    echo "$FILE exists."
    sudo swapoff -a
    sudo rm /swaprpi.img
else 
    echo "$FILE does not exist."
fi


sudo systemctl start ros
sudo systemctl status ros
