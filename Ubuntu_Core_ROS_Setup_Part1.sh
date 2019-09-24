#!/bin/bash

#!!! - If the Raspberry Pi gets rebooted, just
#!!!   re-run this script.

# Make sure we are all up to date
snap refresh

# Install the classic shell (To use apt)
snap install classic --edge --devmode
echo ""
echo "# Part 1 DONE, now onto Part 2"
echo "# Run Ubuntu_Core_ROS_Setup_Part2.sh"
echo ""
# Go into classic shell
sudo classic
