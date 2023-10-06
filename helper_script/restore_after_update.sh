#!/bin/bash


systemctl stop ros

cp /opt/calibration.dat /opt/Poseidon
cp /opt/config.txt /opt/Poseidon

systemctl start ros