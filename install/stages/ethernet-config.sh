#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Ethernet and Wifi Configuration script."
   echo
   echo "Syntax: ethernet-config.sh [options]"
   echo "options:"
   echo "help or h      Print this Help.                  "    
   echo "[hotspot_if]     Hotspot interface name.           $hs_if"
   echo "[hotspot_ssid]   Hotspot SSID.                     $hs_ssid"
   echo "[hotspot_pass]   Hotspot Password.                 $hs_pass"
   echo "[wifi_if]        Wifi interface name.              $wf_if"
   echo "[wifi_ssid]      Wifi SSID.                        $wf_ssid"
   echo "[wifi_pass]      Wifi Password.                    $wf_pass"
   echo
   echo "Command line exemple."
   echo "ethernet-config.sh -help"
   echo "ethernet-config.sh hotspot_if hotspot_ssid hotspot_pass wifi_if wifi_ssid wifi_pass "
   echo "ethernet-config.sh 'wlan1' 'Hydro-B' 'cidco1234' 'wlan0' 'test' 'pass-test'"
   
}

############################################################
# Cfg ethernet                                             #
############################################################
Config()
{
echo "Hotspot interface name.           $hs_if"
echo "Hotspot SSID.                     $hs_ssid"
echo "Hotspot Password.                 $hs_pass"
echo "Wifi interface name.              $wf_if"
echo "Wifi SSID.                        $wf_ssid"
echo "Wifi Password.                    $wf_pass"


echo "Creating WiFi hotspot"
sudo nmcli dev wifi hotspot ifname $hs_if ssid $hs_ssid password $hs_pass | tee -a log.txt
sed -i "s/^hotspotSSID.*/hotspotSSID $hs_ssid/" /opt/Poseidon/config.txt
sudo nmcli con modify Hotspot autoconnect yes
sudo nmcli con modify Hotspot ipv4.addresses 192.168.100.1/24,192.168.100.1

echo "Creating Wifi connection"
sudo nmcli dev wifi connect $wf_ssid password $wf_pass ifname $wf_if

echo "Apply wifi configuration"
sudo nmcli con reload
sudo service network-manager restart

echo "Apply Captive portal"
sudo iptables -t nat -A PREROUTING -p tcp --dport 80 -j DNAT --to-destination 192.168.100.1:80
sudo iptables -t nat -A POSTROUTING -j MASQUERADE

echo "Apply second IP"
sudo bash -c 'cat << EOF2 > /etc/netplan/50-cloud-init.yaml
# This file is generated from information provided by
# the datasource.  Changes to it will not persist across an instance.
# To disable cloud-init s network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
  version: 2
  renderer: NetworkManager
EOF2'

sudo netplan apply
   
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  hs_if=$1
  hs_ssid=$2
  hs_pass=$3
  wf_if=$4
  wf_ssid=$5
  wf_pass=$6

  if [ ! -z "$hs_if" ] && [ ! -z "$hs_ssid" ] && [ ! -z "$hs_pass" ] && [ ! -z "$wf_if" ] && [ ! -z "$wf_ssid" ] && [ ! -z "$wf_pass" ]  
  then
    Config
    exit
  else 
    Help
    exit
  fi
fi






