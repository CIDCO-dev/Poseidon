#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/Poseidon/src/workspace/devel/setup.bash

roslaunch /opt/Poseidon/src/workspace/launch/Binairy/mosaic-x5.launch time_now:=$(date +%Y.%m.%d_%H%M%S)



