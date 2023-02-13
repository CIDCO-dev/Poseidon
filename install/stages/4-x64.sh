#!/bin/bash

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
DEVICES="/dev/ttyS0"
# Other options you want to pass to gpsd
GPSD_OPTIONS="-n"
GPSD_SOCKET="/var/run/gpsd.sock"
EOF2'

sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/

sudo bash -c 'cat << EOF3 > /etc/cron.d/uart
@reboot stty -F /dev/ttyS0 ispeed 38400
EOF3'

echo "[+] Install PPS"

sudo apt-get install pps-tools -y | tee -a log.txt
# sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt'  
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'

echo "[+] Adding roslaunch on boot"
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