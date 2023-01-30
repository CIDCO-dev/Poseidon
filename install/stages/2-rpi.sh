#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Ethernet and Wifi Configuration script."
   echo
   echo "Syntax: 2-rpi.sh [options]"
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
   echo "2-rpi.sh -help"
   echo "2-rpi.sh hotspot_if hotspot_ssid -hotspot_pass -wifi_if -wifi_ssid -wifi_pass -eth_2nd_ip "
   echo "2-rpi.sh 'wlan1' 'Hydro-B' 'cidco1234' 'wlan0' 'test' 'pass-test' '192.168.2.101'"
   
}

############################################################
# Cfg ethernet                                             #
############################################################
Config()
{
echo "[+] Editing uboot"
# removing hang on bonnt cause by data from gps on uart
sudo cp /opt/Poseidon/install/stages/rpi-cfg-files/uboot.env /boot/firmware

echo "[+] Installing RPi.GPIO"
pip3 install RPi.GPIO | tee -a log.txt

echo "Configuring network"
# install network-manager and install as service
sudo apt-get install network-manager -y | tee -a log.txt
sudo systemctl start NetworkManager.service | tee -a log.txt
sudo systemctl enable NetworkManager.service | tee -a log.txt

/opt/Poseidon/install/stages/ethernet-config.sh $hs_if $hs_ssid $hs_pass $wf_if $wf_ssid $wf_pass $snd_ip

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
  snd_ip=$7
  if [ ! -z "$hs_if" ] && [ ! -z "$hs_ssid" ] && [ ! -z "$hs_pass" ] && [ ! -z "$wf_if" ] && [ ! -z "$wf_ssid" ] && [ ! -z "$wf_pass" ] && [ ! -z "$snd_ip" ] 
  then
    Config
    exit
  else 
    Help
    exit
  fi
fi







