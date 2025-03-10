#!/bin/sh

echo -e "\e[35mDownloading GPSD-Cliente\[0m"
cd /opt
sudo git clone https://github.com/CIDCO-dev/gps_umd.git
cd /opt/gps_umd
sudo mv gps* /opt/Poseidon/src/workspace/src/
cd /opt
sudo rm -rd gps_umd



cd /opt/Poseidon/install/stages
sudo ./build-rpi4.sh