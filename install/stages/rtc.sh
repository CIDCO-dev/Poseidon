#!/bin/bash

echo "*********************************************************************"
echo "*                   Enabling the Hardware RTC                       *"
echo "*********************************************************************"   
sudo bash -c 'echo "dtoverlay=i2c-rtc,ds3231" >> /boot/firmware/usercfg.txt'
echo "*********************************************************************"
echo "*                   Romoving the Software RTC                       *"
echo "*********************************************************************"  
sudo apt-get -y remove fake-hwclock
sudo update-rc.d -f fake-hwclock remove
sudo systemctl disable fake-hwclock
sudo hwclock -r