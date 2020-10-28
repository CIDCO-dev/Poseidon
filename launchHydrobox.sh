#!/bin/bash

cd /home/ubuntu/Poseidon/www/webroot
[ ! -d "/record" ] && mkdir record
cd /home/ubuntu/Poseidon


source /opt/ros/melodic/setup.bash
source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash

roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_imagenex852.launch


