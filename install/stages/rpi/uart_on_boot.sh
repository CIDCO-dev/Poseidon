#!/bin/bash
sleep 30
systemctl stop gpsd.socket
systemctl stop gpsd.service
sleep 2
stty -F /dev/ttyAMA0 ispeed 38400
systemctl start gpsd.service
systemctl start gpsd.socket
sleep 20
systemctl restart chrony.service
systemctl restart chronyd.service
