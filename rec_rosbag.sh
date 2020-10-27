#!/bin/bash

source /opt/ros/melodic/setup.bash
source src/workspace/devel/setup.bash

roslaunch src/workspace/launch/rec_rosbag.launch
