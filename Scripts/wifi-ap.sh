sudo apt-get install network-manager
sudo systemctl start NetworkManager.service
sudo systemctl enable NetworkManager.service
nmcli dev wifi hotspot ifname wlan0 ssid Hydro-B password "cidco1234"
