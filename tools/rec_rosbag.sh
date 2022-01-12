#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash

roslaunch /home/ubuntu/Poseidon/src/workspace/launch/Rosbag/rec_rosbag.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

zip -0 -rmj /home/ubuntu/$(date +%Y.%m.%d_%H%M%S)_rosbag.zip /home/ubuntu/*.bag

sudo mv /home/ubuntu/*.zip /home/ubuntu/Poseidon/www/webroot/record/

