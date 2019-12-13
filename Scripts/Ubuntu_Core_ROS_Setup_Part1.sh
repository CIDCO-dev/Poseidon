#!/bin/bash

#!!! - Before running these scripts, make sure the Raspberry Pi
#!!!   has done all its inital install and reboot(s) after
#!!!   Ubuntu Core Installation.

# Make sure we are all up to date
snap refresh

# Install the classic shell (To use apt)
snap install classic --beta --devmode
echo "######################################"
echo "# Part 1 DONE, now onto Part 2       #"
echo "# When the running command is over,  #"
echo "# run Ubuntu_Core_ROS_Setup_Part2.sh #"
echo "######################################"
# Go into classic shell
sudo classic
