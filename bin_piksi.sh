#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/Poseidon/src/workspace/devel/setup.bash

roslaunch /opt/Poseidon/src/workspace/launch/Binairy/piksi.launch time_now:=$(date +%Y.%m.%d_%H%M%S)



