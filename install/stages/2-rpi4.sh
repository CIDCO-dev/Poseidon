#!/bin/sh


echo "[+] Installing RPi.GPIO"
pip3 install RPi.GPIO | tee -a log.txt

echo "Configuring network"
# install network-manager and install as service
sudo apt-get install network-manager -y | tee -a log.txt
sudo systemctl start NetworkManager.service | tee -a log.txt
sudo systemctl enable NetworkManager.service | tee -a log.txt

echo "Rename the RPI WIFI Module at boot"
sudo bash -c 'cat << EOF1 > /etc/udev/rules.d/70-persistent-net.rules
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="brcmfmac", ATTR{dev_id}=="0x0", ATTR{type}=="1", NAME="HotspotWlat"
EOF1'

sudo lshw -c network -xml > /home/ubuntu/ethernet.xml

sudo rm /home/ubuntu/ethernet.xml

#have to detect the corect interface name if rpi wifi module

echo "Rename the RPI WIFI Module during the session"
sudo ip link set wlan1 down
sudo ip link set wlan1 name HotspotWlat
sudo ip link set HotspotWlat up

echo "Creating WiFi hotspot"
sudo nmcli dev wifi hotspot ifname HotspotWlat ssid Hydro-B password "cidco1234" | tee -a log.txt
sudo nmcli con modify Hotspot autoconnect yes
sudo nmcli con modify Hotspot ipv4.addresses 192.168.100.1/24,192.168.100.1
sudo nmcli con reload
sudo service network-manager restart

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
            optional: true
            match:
                name: eth0
            set-name: eth0
            addresses:
              - 192.168.2.101/24
EOF2'

sudo netplan apply

