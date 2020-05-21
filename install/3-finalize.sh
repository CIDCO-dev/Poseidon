#!/bin/bash

echo "[+] Downloading WiringPi"
cd ~/ 
git clone https://github.com/CIDCO-dev/WiringPi-Python.git | tee -a log.txt
cd WiringPi/
./build | tee -a log.txt

echo "[+] Configuring time system"
cd ~/
git clone https://github.com/CIDCO-dev/RaspberryPi.git | tee -a log.txt

echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=pi3-miniuart-bt" >> /boot/firmware/config.txt' | tee -a log.txt
sudo systemctl mask serial-getty@ttyAMA0.service | tee -a log.txt

echo "[+] Install GPSD"
sudo apt-get install gpsd gpsd-clients -y | tee -a log.txt
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

echo "[+] Install PPS"

sudo apt-get install pps-tools -y  >> log.txt 2> /dev/null
sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt'  
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'

echo "[+] Install Chrony"

sudo apt-get install chrony -y  >> log.txt 2> /dev/null

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

