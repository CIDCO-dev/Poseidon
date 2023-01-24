#!/bin/bash


echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=disable-bt" >> /boot/firmware/usercfg.txt' | tee -a log.txt
sudo systemctl mask serial-getty@ttyAMA0.service | tee -a log.txt
usrmod -G dialout ubuntu

echo "[+] Install GPSD"
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
DEVICES="/dev/ttyAMA0"
# Other options you want to pass to gpsd
GPSD_OPTIONS="-n"
GPSD_SOCKET="/var/run/gpsd.sock"
EOF2'

sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/

echo "[+] config /boot"
sudo sed 's/intfc:uart2=off/intfc:uart2=on/g /boot/hw_intfc.conf' > /boot/hw_intfc.conf
sudo sed 's/intfc:i2c7=off/intfc:i2c7=on/g /boot/hw_intfc.conf' > /boot/hw_intfc.conf

sudo sed 's/console-ttyS2,1500000n8//g /boot/extlinux/extlinux.conf' > /boot/extlinux/extlinux.conf


echo "[+] Adding roslaunch on boot"
sudo bash -c 'cat << EOF3 > /etc/systemd/system/ros.service
[Unit]
Description=Launch ROS on boot.
After=gpsd.service hwrtc.service

[Service]
Type=simple
ExecStart=/home/ubuntu/Poseidon/launchROSService.sh

[Install]
WantedBy=multi-user.target
EOF3'

sudo chmod 755 /etc/systemd/system/ros.service
sudo systemctl enable ros
echo "sudo systemctl start ros"

echo "[+] Adding Uart config on boot"
sudo bash -c 'cat << EOF5 > /etc/systemd/system/uart.service
[Unit]
Description=Launch Uart config on boot.
Before=gpsd.service

[Service]
Type=simple
ExecStart=/home/ubuntu/Poseidon/service/uart_on_boot.sh

[Install]
WantedBy=multi-user.target
EOF5'

sudo chmod 755 /etc/systemd/system/uart.service
sudo systemctl enable uart


