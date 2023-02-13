#!/bin/bash


echo -e "\[35m[+] Configuring UART\[0m"
sudo systemctl mask serial-getty@ttyS2.service | tee -a log.txt
usermod -G dialout ubuntu

echo -e "\[35m[+] Install GPSD\[0m"
sudo apt-get install gpsd gpsd-clients libgps-dev -y | tee -a log.txt
sudo cp /etc/default/gpsd "/etc/default/gpsd.bak$(date +"%Y%m%d_%H%M%S")"

sudo bash -c 'cat << EOF2 > /etc/default/gpsd
# Default settings for the gpsd init script and the hotplug wrapper.
# Start the gpsd daemon automatically at boot time
START_DAEMON="true"
# Use USB hotplugging to add new USB devices automatically to the daemon
USBAUTO="true"
# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="/dev/ttyS2"
# Other options you want to pass to gpsd
GPSD_OPTIONS="-n"
GPSD_SOCKET="/var/run/gpsd.sock"
EOF2'

sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/

echo -e "\[35m[+] config /boot\[0m"
sudo sed -i 's/intfc:uart2=off/intfc:uart2=on/g' /boot/hw_intfc.conf
sudo sed -i 's/intfc:i2c7=off/intfc:i2c7=on/g' /boot/hw_intfc.conf
sudo sed -i 's/intfc:dtoverlay=console-on-ttyS2/#intfc:dtoverlay=console-on-ttyS2/g' /boot/hw_intfc.conf

sudo sed -i 's/ console=ttyS2,1500000n8//g' /boot/extlinux/extlinux.conf
sudo sed -i 's/ console=ttyFIQ0,1500000n8// g' /boot/extlinux/extlinux.conf 
sudo sed -i 's/rw //g' /boot/extlinux/extlinux.conf 
sudo sed -i 's/init=/sbin/init //g' /boot/extlinux/extlinux.conf 


sudo sed -i 's/ttyAMA0/ttyS2/g'/opt/Poseidon/service/uart_on_boot.sh

echo -e "\[35m[+] Adding roslaunch on boot\[0m"
sudo bash -c 'cat << EOF3 > /etc/systemd/system/ros.service
[Unit]
Description=Launch ROS on boot.
After=gpsd.service hwrtc.service

[Service]
Type=simple
ExecStart=/opt/Poseidon/launchROSService.sh

[Install]
WantedBy=multi-user.target
EOF3'

sudo chmod 755 /etc/systemd/system/ros.service
sudo systemctl enable ros
echo "sudo systemctl start ros"

echo -e "\[35m[+] Adding Uart config on boot\[0m"
sudo bash -c 'cat << EOF5 > /etc/systemd/system/uart.service
[Unit]
Description=Launch Uart config on boot.
Before=gpsd.service

[Service]
Type=simple
ExecStart=/opt/Poseidon/service/uart_on_boot.sh

[Install]
WantedBy=multi-user.target
EOF5'

sudo chmod 755 /etc/systemd/system/uart.service
sudo systemctl enable uart

echo -e "\[35m[+] editing launch files\[0m"
sudo sed -i 's/i2c-1/i2c-7/g' /opt/Poseidon/src/workspace/launch/Hydrobox/*.*

echo -e "\[35m[+] editing uart service files\[0m"
sudo sed -i 's/AMA0/S2/g' /opt/Poseidon/service/*.*