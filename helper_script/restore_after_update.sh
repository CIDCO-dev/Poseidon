#!/bin/bash


systemctl stop ros

cp /opt/Poseidon.backup/calibration.dat /opt/Poseidon
cp /opt/Poseidon.backup/config.txt /opt/Poseidon
cp /opt/Poseidon.backup/uart_on_boot.sh /opt/Poseidon/service
cp /opt/Poseidon.backup/launchROSSServoce.sh /opt/Poseidon

now=`date +"%Y-%m-%d"`

zip -r /opt/Poseidon/www/webroot/record/Poseidon.config.${now}.zip /opt/Poseidon.backup

systemctl start ros
