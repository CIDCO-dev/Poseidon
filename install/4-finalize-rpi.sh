#!/bin/bash

echo "[+] Downloading WiringPi"
cd ~/ 
git clone https://github.com/CIDCO-dev/WiringPi.git | tee -a log.txt
cd WiringPi/
./build | tee -a log.txt

echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=pi3-miniuart-bt" >> /boot/firmware/config.txt' | tee -a log.txt
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

sudo anet.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc
pt-get install pps-tools -y | tee -a log.txt
sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt'  
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'

echo "[+] Firmware switch"
sudo bash -c 'cat << EOF2 > /boot/firmware/config.txt
# Please DO NOT modify this file; if you need to modify the boot config, the
# >usercfg.txt> file is the place to include user changes. Please refer to
# the README file for a description of the various configuration files on
# the boot partition.

# The unusual ordering below is deliberate; older firmwares (in particular the
# version initially shipped with bionic) don<t understand the conditional
# sections below and simply ignore them. The Pi4 doesn<t boot at all with
# firmwares this old so it<s safe to place at the top. Of the Pi2 and Pi3, the
# Pi3 uboot happens to work happily on the Pi2, so it needs to go at the bottom
# to support old firmwares.

pi4
kernel=uboot_rpi_4.bin
max_framebuffers=2

pi2
kernel=uboot_rpi_2.bin
net.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc

pi3
kernel=uboot_rpi_3.bin

all
arm_64bit=1
device_tree_address=0x03000000
kernel=vmlinuz
initramfs initrd.img followkernel

# The following settings are >defaults> expected to be overridden by the
# included configuration. The only reason they are included is, again, to
# support old firmwares which don<t understand the >include> command.

enable_uart=1
cmdline=nobtcmd.txt

include syscfg.txt
include usercfg.txt
dtoverlay=pi3-miniuart-bt
dtoverlay=pps-gpio,gpiopin=4
EOF2'

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
ExecStart=source /opt/ros/melodic/setup.bash && source /home/ubuntu/Poseidon/src/workspace/devel/setup.bash && /usr/bin/bash /home/ubuntu/Poseidon/launchHydrobox.sh

[Install]
WantedBy=multi-user.target
EOF3'

chmod 755 /etc/systemd/system/ros.service
systemctl enable ros
