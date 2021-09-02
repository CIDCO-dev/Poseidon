sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-usb-serial.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A500D8GK",SYMLINK+="imu"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="AC00Y2Y7" SYMLINK+="sonar"
EOF'
