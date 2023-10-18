#!/bin/bash


systemctl stop ros

[ ! -d "/opt/Poseidon.backup" ] && mkdir /opt/Poseidon.backup

cp /opt/Poseidon/launchROSSServoce.sh /opt/Poseidon.backup
cp /opt/Poseidon/calibration.dat /opt/Poseidon.backup
cp /opt/Poseidon/config.txt /opt/Poseidon.backup
cp /opt/Poseidon/service/uart_on_boot.sh /opt/Poseidon.backup

