sudo echo    wifis: >> /etc/netplan/0-cloud-init.yaml
sudo echo        wlan0: >> /etc/netplan/0-cloud-init.yaml
sudo echo            optional: true >> /etc/netplan/0-cloud-init.yaml
sudo echo            access-points: >> /etc/netplan/0-cloud-init.yaml
sudo echo                "Hydro-B": >> /etc/netplan/0-cloud-init.yaml
sudo echo                    password: "test" >> /etc/netplan/0-cloud-init.yaml
sudo echo            dhcp4: no >> /etc/netplan/0-cloud-init.yaml
sudo echo            dhcp6: no >> /etc/netplan/0-cloud-init.yaml
sudo echo            addresses: [192.168.0.1/24] >> /etc/netplan/0-cloud-init.yaml
sudo echo            gateway4: 192.168.0.1 >> /etc/netplan/0-cloud-init.yaml


