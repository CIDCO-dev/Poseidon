#!/bin/bash

source /opt/ros/melodic/setup.bash
source /opt/Poseidon/src/workspace/devel/setup.bash

roslaunch /opt/Poseidon/src/workspace/launch/Rosbag/rec_rosbag.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

zip -0 -rmj /opt/$(date +%Y.%m.%d_%H%M%S)_rosbag.zip /opt/*.bag

sudo mv /opt/*.zip /opt/Poseidon/www/webroot/record/

