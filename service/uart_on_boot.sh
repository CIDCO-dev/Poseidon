#!/bin/bash

# GNSS uart speed
stty -F /dev/ttyAMA0 ispeed 38400

# Sonar uart speed
stty -F /dev/ttyUSB0 ispeed 4800