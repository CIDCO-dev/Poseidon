#!/bin/bash

echo "[+] Downloading WiringPi"
cd ~/ 
git clone https://github.com/CIDCO-dev/WiringPi.git | tee -a log.txt
cd WiringPi/
./build | tee -a log.txt

echo "[+] Configuring Inertial Sense SDK"
cd ~/Poseidon/src/workspace/src/inertial_sense_ros/
mkdir lib
cd lib
git clone https://github.com/inertialsense/InertialSenseSDK | tee -a log.txt

echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=pi3-miniuart-bt" >> /boot/firmware/config.txt' | tee -a log.txt
sudo systemctl mask serial-getty@ttyAMA0.service | tee -a log.txt

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

echo "[+] Install Chrony"

sudo apt-get install chrony -y | tee -a log.txt

sudo bash -c 'cat << EOF2 > /etc/chrony/chrony.conf
# PPS: /dev/pps0: Kernel-mode PPS ref-clock for the precise seconds
refclock  PPS /dev/pps0  refid PPS  precision 1e-9  lock NMEA  poll 3  trust  prefer
# SHM(2), gpsd: PPS data from shared memory provided by gpsd
#refclock  SHM 2  refid PPSx  precision 1e-9  poll 3  trust
# SOCK, gpsd: PPS data from socket provided by gpsd
#refclock  SOCK /var/run/chrony.pps0.sock  refid PPSy  precision 1e-9  poll 3  trust
# SHM(0), gpsd: NMEA data from shared memory provided by gpsd
refclock  SHM 0  refid NMEA  precision 1e-3  offset 0.5  delay 0.2  poll 3  trust  require
# any NTP clients are allowed to access the NTP server.
allow
# allows to appear synchronised to NTP clients, even when it is not.
local
# Stratum1 Servers
# https://www.meinbergglobal.com/english/glossary/public-time-server.htm
#
## Physikalisch-Technische Bundesanstalt (PTB), Braunschweig, Germany
#server  ptbtime1.ptb.de  iburst  noselect
#server  ptbtime2.ptb.de  iburst  noselect
#server  ptbtime3.ptb.de  iburst  noselect
#
## Royal Observatory of Belgium
#server  ntp1.oma.be  iburst  noselect
#server  ntp2.oma.be  iburst  noselect
#
## Unizeto Technologies S.A., Szczecin, Polska
#server  ntp.certum.pl  iburst  noselect
#
## SP Swedish driftfileNational Testing and Research Institute, Boras, Sweden
#server  ntp2.sp.se  iburst  noselect
# Other NTP Servers
#pool  de.pool.ntp.org  iburst  noselect
# This directive specify the location of the file containing ID/key pairs for
# NTP authentication.
keyfile /etc/chrony/chrony.keys
# This directive specify the file into which chronyd will store the rate
# information.
driftfile /var/lib/chrony/chrony.drift
# Uncomment the following line to turn logging on.
#log tracking measurements statistics
# Log files location.
logdir /var/log/chrony
# Stop bad estimates upsetting machine clock.
maxupdateskew 100.0
# This directive tells 'chronyd' to parse the 'adjtime' file to find out if the
# real-time clock keeps local time or UTC. It overrides the 'rtconutc' directive.
hwclockfile /etc/adjtime
# This directive enables kernel synchronisation (every 11 minutes) of the
# real-time clock. Note that it can’t be used along with the 'rtcfile' directive.
rtcsync
# Step the system clock instead of slewing it if the adjustment is larger than
# one second, but only in the first three clock updates.
makestep 1 3
EOF2'

echo "[+] Enabling SPI"
sudo bash -c 'cat << EOF2 > /etc/udev/rules.d/50-spi.rules
KERNEL=="spidev*", GROUP="dialout", MODE="0664"
EOF2'

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

echo "[+] Downloading Rtklib"
cd ~/ 
git clone https://github.com/CIDCO-dev/RTKLIB.git | tee -a log.txt
cd RTKLIB/app
sudo chmod +x makeall.sh
./makeall.sh | tee -a log.txt

