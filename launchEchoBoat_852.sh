#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash

roslaunch /home/ubuntu/Poseidon/src/workspace/launch/echoboat_imaginex852.launch time_now:=$(date +%Y.%m.%d_%H%M%S)



