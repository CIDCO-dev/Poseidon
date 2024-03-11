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
   echo "[eth_2nd_ip]     Wired ethernet second ip address. $snd_ip"
   echo
   echo "Command line exemple."
   echo "ethernet-config.sh -help"
   echo "ethernet-config.sh hotspot_if hotspot_ssid hotspot_pass wifi_if wifi_ssid wifi_pass eth_2nd_ip "
   echo "ethernet-config.sh 'wlan1' 'Hydro-B' 'cidco1234' 'wlan0' 'test' 'pass-test' '192.168.2.101'"
   
}

############################################################
# Cfg ethernet                                             #
############################################################
Config()
{
echo "Hotspot interface name.           $hs_if"


echo "Disabling WiFi hotspot"
sudo nmcli connection down id "Hotspot" | tee -a log.txt


echo "Creating Wifi connection"
nm_connection_file="/etc/NetworkManager/system-connections/Hotspot.nmconnection"
old_interface=$(awk -F= '/interface-name/{print $2}' $nm_connection_file)
new_interface=$hs_if
sudo sed -i "s/interface-name=$old_interface/interface-name=$new_interface/g" $nm_connection_file


echo "Apply wifi configuration"
sudo nmcli con reload
sudo service network-manager restart

echo "Enabling WiFi hotspot"
sudo nmcli connection up id "Hotspot" | tee -a log.txt
   
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  hs_if=$1
 
  if [ ! -z "$hs_if" ] 
  then
    Config
    exit
  else 
    Help
    exit
  fi
fi






