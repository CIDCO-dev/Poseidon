echo "Creating WiFi hotspot"
sudo nmcli dev wifi hotspot ifname wlan1 ssid Hydro-B password "cidco1234" | tee -a log.txt
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
    wifis:
        wlan0:
            dhcp4: true
            access-point:
                "Phyl0":
                    password: "4012401240124012"

EOF2'
