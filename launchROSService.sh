#!/bin/bash

# Used to call a launch file as a service on boot

source /opt/ros/noetic/setup.bash
source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash

# Echoboat configuration with imaginex 852
#roslaunch /home/ubuntu/Poseidon/src/workspace/launch/echoboat_imaginex852.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock conbfiguration with Piksi and imaginex 852
#roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_imagenex852_piksi.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock configuration with Piksi and nmea sonar
#roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_nmeadevice_piksi.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock configuration with X5 and imaginex 852
#roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_imagenex_x5.launch time_now:=$(date +%Y.%m.%d_%H%M%S)


# Hydroblock configuration with X5 and nmea sonar
#roslaunch /home/ubuntu/Poseidon/src/workspace/launch/hydrobox_nmeadevice_x5.launch time_now:=$(date +%Y.%m.%d_%H%M%S)


# Lowrance simulator
#roslaunch  /home/ubuntu/Poseidon/src/workspace/launch/lowrance_simulator.launch

# Dummy simulator
roslaunch /home/ubuntu/Poseidon/src/workspace/launch/dummy_simulator.launch
