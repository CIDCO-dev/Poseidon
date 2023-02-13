#!/bin/bash
echo -e "\[35m[+] install opencv\[0m"
sudo apt-get install libopencv-dev -y | tee -a log.txt

echo -e "\[35m[+] install exiv2\[0m"
sudo apt-get install libexiv2-dev -y | tee -a log.txt

echo -e "\[35m[+] Configuring Inertial Sense SDK\[0m"
cd /opt/Poseidon/src/workspace/src/inertial_sense_ros/
mkdir lib
cd lib
git clone https://github.com/inertialsense/InertialSenseSDK | tee -a log.txt

echo -e "\[35m[+] Install Chrony\[0m"

sudo apt-get install chrony -y | tee -a log.txt

sudo bash -c 'cat << EOF2 > /etc/chrony/chrony.conf
# PPS: /dev/pps0: Kernel-mode PPS ref-clock for the precise seconds
refclock  PPS /dev/pps0  refid PPS  precision 1e-9  poll 3  trust 
# SHM(2), gpsd: PPS data from shared memory provided by gpsd
#refclock  SHM 2  refid PPSx  precision 1e-9  poll 3  trust
# SOCK, gpsd: PPS data from socket provided by gpsd
#refclock  SOCK /var/run/chrony.pps0.sock  refid PPSy  precision 1e-9  poll 3  trust
# SHM(0), gpsd: NMEA data from shared memory provided by gpsd
refclock  SHM 0  refid NMEA  precision 1e-3  offset 0.5  delay 0.2  poll 3  trust  prefer
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

echo -e "\[35m[+] Downloading Rtklib\[0m"
cd ~/ 
git clone https://github.com/CIDCO-dev/RTKLIB.git | tee -a log.txt
cd RTKLIB/app
sudo chmod +x makeall.sh
./makeall.sh | tee -a log.txt



