#!/bin/bash

echo "[+] Downloading WiringPi"
cd ~/ 
git clone https://github.com/CIDCO-dev/WiringPi.git | tee -a log.txt
cd WiringPi/
./build | tee -a log.txt

echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt' | tee -a log.txt
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

sudo bash -c 'cat << EOF3 > /etc/cron.d/uart
@reboot stty -F /dev/ttyAMA0 ispeed 38400
EOF3'

echo "[+] Install PPS"
sudo apt-get install pps-tools -y | tee -a log.txt

sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt'  
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'

echo "[+] Config uart"
sudo bash -c 'cat << EOF2 > /boot/firmware/nobtcmd.txt
net.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc
EOF2'

echo "[+] Adding roslaunch on boot"
sudo bash -c 'cat << EOF3 > /etc/systemd/system/ros.service
[Unit]
Description=Launch ROS on boot.

[Service]
Type=simple
ExecStart=/home/ubuntu/Poseidon/launchHydrobox.sh

[Install]
WantedBy=multi-user.target
EOF3'

sudo chmod 755 /etc/systemd/system/ros.service
sudo systemctl enable ros
sudo systemctl start ros
