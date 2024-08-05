#!/bin/bash

# Used to call a launch file as a service on boot

source /opt/ros/noetic/setup.bash
source /opt/Poseidon/src/workspace/devel/setup.bash

########################
#      Echoboat        #
########################

# Echoboat configuration with imaginex 852
#roslaunch /opt/Poseidon/src/workspace/launch/Echoboat/echoboat_imaginex852.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Echoboat configuration with out sonar
#roslaunch /opt/Poseidon/src/workspace/launch/Echoboat/echoboat.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

###########################
#      Hydroblock  RPI    #
###########################

# Hydroblock RPI configuration with imaginex 852 and Piksi and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_imagenex852_piksi_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with imaginex 852 and piksi and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_imagenex852_piksi_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with imaginex 852 and X5 and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_imagenex852_x5_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with imaginex 852 and X5 and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_imagenex852_x5_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock configuration with Imagenex 852 sonar and ZED-F9P and bno055
roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_imagenex852_Zed-F9P_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

###

# Hydroblock RPI configuration with nmea sonar and piksi and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_nmeadevice_piksi_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with nmea sonar and piksi and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_nmeadevice_piksi_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with nmea sonar and X5 and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_nmeadevice_x5_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock RPI configuration with nmea sonar and X5 and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_nmeadevice_x5_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock configuration with nmea sonar and ZED-F9P and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rpi_nmeadevice_ZED-F9P_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)



############################
#      Hydroblock  Rock    #
############################

# Hydroblock Rock configuration with imaginex 852 and Piksi and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_imagenex852_piksi_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with imaginex 852 and piksi and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_imagenex852_piksi_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with imaginex 852 and X5 and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_imagenex852_x5_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with imaginex 852 and X5 and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_imagenex852_x5_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

###

# Hydroblock Rock configuration with nmea sonar and piksi and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_nmeadevice_piksi_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with nmea sonar and piksi and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_nmeadevice_piksi_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with nmea sonar and X5 and null
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_nmeadevice_x5_null.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with nmea sonar and X5 and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_nmeadevice_x5_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)

# Hydroblock Rock configuration with nmea sonar and ZED-F9P and bno055
#roslaunch /opt/Poseidon/src/workspace/launch/Hydrobox/hydrobox_rock_nmeadevice_ZED-F9P_bno055.launch time_now:=$(date +%Y.%m.%d_%H%M%S)


########################
#      Simulator       #
########################

# Lowrance simulator
#roslaunch  /opt/Poseidon/src/workspace/launch/Simulator/lowrance_simulator.launch

# Dummy simulator
#roslaunch /opt/Poseidon/src/workspace/launch/Simulator/dummy_simulator.launch

########################
#    Configuration     #
########################

# Dummy simulator
#roslaunch /opt/Poseidon/src/workspace/launch/Configuration/Config_ZED-F9P.launch

# BNO-055 Calibration
#roslaunch /opt/Poseidon/src/workspace/launch/Configuration/Calibration_BNO055.launch
