#!/bin/sh

echo "Configuring network"
# install network-manager and install as service
sudo apt-get install network-manager -y | tee -a log.txt
sudo systemctl start NetworkManager.service | tee -a log.txt
sudo systemctl enable NetworkManager.service | tee -a log.txt

