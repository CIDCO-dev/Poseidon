#!/bin/bash


if [ -z "$STY" ]; then
    echo -e "\e[31mT'hits script need to be launch from 'screen' session.\e[0m"
    exit 1
fi

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ssid>"
    exit 1
fi

SSID=$1


echo -e "\e[35m--> Old SSID\e[0m"
nmcli con show Hotspot | grep wireless.ssid
echo -e "\e[35m--> Update the SSID\e[0m"
sudo nmcli con modify Hotspot 802-11-wireless.ssid "$SSID"
echo -e "\e[35m--> New SSID\e[0m"
nmcli con show Hotspot | grep wireless.ssid
echo -e "\e[35m--> Restart the wifi\e[0m"
sudo nmcli con down Hotspot
sudo nmcli con up Hotspot
