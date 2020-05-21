#!/bin/sh

echo "Configuring network"
# install network-manager and install as service
sudo apt-get install network-manager -y | tee -a log.txt
sudo systemctl start NetworkManager.service | tee -a log.txt
sudo systemctl enable NetworkManager.service | tee -a log.txt

echo "Creating WiFi hotspot"
sudo nmcli dev wifi hotspot ifname wlan0 ssid Hydro-B password "cidco1234" | tee -a log.txt
sudo nmcli con modify Hotspot autoconnect yes
sudo nmcli con modify Hotspot ipv4.addresses 192.168.1.1/24,192.168.1.1
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
#            match:
#                macaddress: b8:27:eb:47:75:13
            set-name: eth0
            addresses:
              - 192.168.2.101/24
EOF2'

sudo netplan apply

echo "[+] Installing web server"
sudo apt install lighttpd -y | tee -a log.txt

sudo systemctl start lighttpd.service | tee -a log.txt
sudo systemctl enable lighttpd.service | tee -a log.txt

echo "[+] Installing websocket library"
sudo apt install libwebsocketpp-dev

echo "[+] Installing web UI"
sudo mkdir -p /var/www
sudo chmod +644 /var/www
cd /var/www/
git clone https://github.com/Ddoiron-cidco/Poseidon_web.git
sudo rm -r -d html  | tee -a log.txt
mv Poseidon_web html

