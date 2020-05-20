#!/bin/bash

#update the apps
echo --------------------
echo Updating Deb.
echo --------------------
sudo apt update >> log.txt 2> /dev/null

#upgrade the dist
echo --------------------
echo Updating Dist.
echo --------------------
sudo apt dist-upgrade -y >> log.txt 2> /dev/null

#upgrade the apps
echo --------------------
echo Updating Apps.
echo --------------------
sudo apt upgrade -y >> log.txt 2> /dev/null

# Install utilities with the YES answer to apt prompt
echo --------------------
echo Installing Utilities.
echo --------------------
sudo apt install gcc python3-dev python3-pip python3-setuptools git curl -y >> log.txt 2> /dev/null

# Install RPi.GPIO
pip3 install RPi.GPIO >> log.txt 2> /dev/null
# Now ROS Installation
echo --------------------
echo Installing ROS.
echo --------------------
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' >> log.txt 2> /dev/null
# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 >> log.txt 2> /dev/null
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add - >> log.txt 2> /dev/null

# Actual Installation
sudo apt-get update >> log.txt 2> /dev/null

#sudo apt install ros-kinetic-ros-base g++ -y >> log.txt
sudo apt install ros-melodic-ros-base g++ -y >> log.txt 2> /dev/null

# Initialize rosdep
echo --------------------
echo Initialize ROSdep
echo --------------------
rosdep init >> log.txt 2> /dev/null
echo --------------------
echo Updating ROSdep
echo --------------------
rosdep update >> log.txt 2> /dev/null

echo --------------------
echo Installing Tools
echo --------------------
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y >> log.txt 2> /dev/null

echo --------------------
echo Installing Network-Manager
echo --------------------
sudo apt-get install network-manager -y >> log.txt 2> /dev/null
echo --------------------
echo Installing Network Service
echo --------------------
sudo systemctl start NetworkManager.service >> log.txt 2> /dev/null
sudo systemctl enable NetworkManager.service >> log.txt 2> /dev/null
echo --------------------
echo Creating Wifi HotSpot
echo --------------------
sudo nmcli dev wifi hotspot ifname wlan0 ssid Hydro-B password "cidco1234" >> log.txt 2> /dev/null
sudo nmcli con modify Hotspot autoconnect yes
sudo nmcli con modify Hotspot ipv4.addresses 192.168.1.1/24,192.168.1.1
sudo nmcli con reload
sudo service network-manager restart
echo --------------------
echo Configuring network interface
echo --------------------
sudo bash -c 'cat << EOF2 > /etc/netplan/50-cloud-init.yaml
# This file is generated from information provided by
# the datasource.  Changes to it will not persist across an instance.
# To disable cloud-init s network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: true
            match:
                macaddress: b8:27:eb:47:75:13
            set-name: eth0
            addresses:
              - 192.168.2.101/24
EOF2'

sudo netplan apply

echo --------------------
echo Downloading Poseidon
echo --------------------

cd ~/
sudo apt install libgps-dev ros-melodic-rosbridge-server
git clone https://github.com/Ddoiron-cidco/Poseidon.git

echo --------------------
echo Installing web server
echo --------------------
sudo apt install lighttpd -y >> log.txt 2> /dev/null
echo --------------------
echo Installing web service
echo --------------------
sudo systemctl start lighttpd.service >> log.txt 2> /dev/null
sudo systemctl enable lighttpd.service >> log.txt 2> /dev/null
echo --------------------
echo Installing web socket
echo --------------------
sudo apt install libwebsocketpp-dev


#installer un captive portal
echo --------------------
echo Installing web site
echo --------------------

sudo bash -c 'cat << EOF2 > /etc/lighttpd/lighttpd.conf
server.modules = (
	"mod_access",
	"mod_alias",
	"mod_compress",
 	"mod_redirect",
)

server.document-root        = "/home/ubuntu/Poseidon/www/webroot"
server.upload-dirs          = ( "/var/cache/lighttpd/uploads" )
server.errorlog             = "/var/log/lighttpd/error.log"
server.pid-file             = "/var/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = 80


index-file.names            = ( "index.php", "index.html", "index.lighttpd.html" )
url.access-deny             = ( "~", ".inc" )
static-file.exclude-extensions = ( ".php", ".pl", ".fcgi" )

compress.cache-dir          = "/var/cache/lighttpd/compress/"
compress.filetype           = ( "application/javascript", "text/css", "text/html", "text/plain" )

# default listening port for IPv6 falls back to the IPv4 port
## Use ipv6 if available
#include_shell "/usr/share/lighttpd/use-ipv6.pl " + server.port
include_shell "/usr/share/lighttpd/create-mime.assign.pl"
include_shell "/usr/share/lighttpd/include-conf-enabled.pl"
EOF2'
sudo systemctl reload lighttpd.service

echo --------------------
echo Downloading WiringPi
echo --------------------
cd ~/
git clone https://github.com/Ddoiron-cidco/WiringPi.git  >> log.txt 2> /dev/null
cd WiringPi/
./build  >> log.txt 2> /dev/null

echo --------------------
echo Connfiguring time system
echo --------------------
cd ~/
git clone https://github.com/Ddoiron-cidco/RaspberryPi.git  >> log.txt 2> /dev/null

echo --------------------
echo Connfiguring uart
echo --------------------

sudo bash -c 'echo "dtoverlay=pi3-miniuart-bt" >> /boot/firmware/config.txt'  >> log.txt 2> /dev/null
sudo systemctl mask serial-getty@ttyAMA0.service  >> log.txt 2> /dev/null


echo --------------------
echo Install GPSD
echo --------------------

sudo apt-get install gpsd gpsd-clients -y  >> log.txt 2> /dev/null
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

echo --------------------
echo Install PPS Suport tools
echo --------------------

sudo apt-get install pps-tools -y  >> log.txt 2> /dev/null
sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt'  
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'  
echo --------------------
echo Install Chrony
echo --------------------

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

echo --------------------
echo Spi Rule
echo --------------------

sudo bash -c 'cat << EOF2 > /etc/udev/rules.d/50-spi.rules
KERNEL=="spidev*", GROUP="dialout", MODE="0664"
EOF2'

echo --------------------
echo End of script 
echo --------------------

sudo reboot
