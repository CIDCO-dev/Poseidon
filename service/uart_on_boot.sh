#!/bin/bash

# GNSS uart speed
stty -F /dev/gnss ispeed 38400

# Sonar uart speed 
#9600bps = default speed for nmea test
stty -F /dev/sonar ispeed 9600