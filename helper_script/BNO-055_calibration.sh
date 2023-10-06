#!/bin/bash



source /opt/ros/noetic/setup.bash
source /opt/Poseidon/src/workspace/devel/setup.bash

systemctl stop ros
rm /opt/Poseidon/calibration.dat

# BNO-055 Calibration
roslaunch /opt/Poseidon/src/workspace/launch/Configuration/Calibration_BNO055.launch

systemctl start ros