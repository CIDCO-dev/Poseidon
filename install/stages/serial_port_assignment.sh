sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-usb-serial.rules
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="A500D8GK",SYMLINK+="imu" 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001", SYMLINK+="sonar"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1546",ATTRS{idProduct}=="01a9", SYMLINK+="gnss"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483",ATTRS{idProduct}=="a217", SYMLINK+="sonar"
EOF'
