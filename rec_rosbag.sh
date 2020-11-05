#!/bin/bash

source /opt/ros/melodic/setup.bash
source src/workspace/devel/setup.bash

roslaunch src/workspace/launch/rec_rosbag.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

zip -9 -rm /home/ubuntu/$(date +%Y.%m.%d_%H%M%S)_rosbag.zip /home/ubuntu/*.bag

sudo mv /home/ubuntu/*.zip /home/ubuntu/Poseidon/www/webroot/record/

