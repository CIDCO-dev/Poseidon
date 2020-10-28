#!/bin/bash

[ ! -d "~/Poseidon/www/webroot/record" ] && mkdir -p "`/Poseidon/www/webroot/record"

source /opt/ros/melodic/setup.bash
source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash

roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_imagenex852.launch


