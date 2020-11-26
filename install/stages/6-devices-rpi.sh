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

sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-usb-serial.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A500D8GK",SYMLINK+="imu"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="AC00Y2Y7" SYMLINK+="sonar"
EOF'
