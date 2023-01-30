#!/bin/sh

echo "[+] Enabling SPI"
sudo bash -c 'cat << EOF2 > /etc/udev/rules.d/50-spi.rules
KERNEL=="spidev*", GROUP="dialout", MODE="0664"
EOF2'

echo "[+] Adding user to i2c group"
sudo usermod -a -G i2c ubuntu

echo "[+] Remapping hydrographic devices"

#Intertial sense IMUs use a FT232-based USB-serial chip, and thus appear as such through idVendor 0x0403 and idProduct 0x$
#rs232 sonar dongle appears as PL2303 with idVendor 0x067b and idProduct 0x2303

sudo bash -c 'cat << EOF > /etc/udev/rules.d/9sudo 9-usb-serial.rules
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A500D8GK",SYMLINK+="imu"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001", SYMLINK+="sonar"
EOF'

cd /opt/Poseidon/src/workspace/src/

echo "Download mavros"
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs ros-noetic-control-toolbox -y

sudo sed -i 's/refclock  PPS/#refclock  PPS/g' /etc/chrony/chrony.conf
