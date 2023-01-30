#!/bin/bash

sudo systemctl stop ros

echo "[+] Building Poseidon"

cd /opt/Poseidon/src/workspace
source /opt/ros/noetic/setup.bash
catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;libmavconn;mavros_extras;echoboat_odometry;mavros_msgs;test_mavros;video_recorder"

[ ! -d "/opt/Poseidon/www/webroot/record" ] && mkdir /opt/Poseidon/www/webroot/record

sudo systemctl start ros
sudo systemctl status ros
