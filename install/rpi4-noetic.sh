#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Install script for Raspberry 4 with ubuntu 20."
   echo
   echo "Syntax: rpi4-noetic.sh [options]"
   echo "options:"
   echo "help or h      Print this Help.                  "    
   echo "[hotspot_if]     Hotspot interface name.           $hs_if"
   echo "[hotspot_ssid]   Hotspot SSID.                     $hs_ssid"
   echo "[hotspot_pass]   Hotspot Password.                 $hs_pass"
   echo "[wifi_if]        Wifi interface name.              $wf_if"
   echo "[wifi_ssid]      Wifi SSID.                        $wf_ssid"
   echo "[wifi_pass]      Wifi Password.                    $wf_pass"
   echo "[eth_2nd_ip]     Wired ethernet second ip address. $snd_ip"
   echo "[rtc]            Enable Hardware RTC and disable Software RTC"
   echo
   echo "Command line exemple."
   echo "rpi4-noetic.sh -help"
   echo "For Software RTC"
   echo "rpi4-noetic.sh hotspot_if hotspot_ssid -hotspot_pass -wifi_if -wifi_ssid -wifi_pass -eth_2nd_ip "
   echo "rpi4-noetic.sh 'wlan1' 'Hydro-B' 'cidco1234' 'wlan0' 'test' 'pass-test' '192.168.2.101'"
   echo "For Hardware RTC"
   echo "rpi4-noetic.sh hotspot_if hotspot_ssid hotspot_pass wifi_if wifi_ssid wifi_pass eth_2nd_ip rtc"
   echo "rpi4-noetic.sh 'wlan1' 'Hydro-B' 'cidco1234' 'wlan0' 'test' 'pass-1234' '192.168.2.101' 'rtc'"
   
}

############################################################
# Cfg ethernet                                             #
############################################################
Config()
{
/opt/Poseidon/install/stages/1-base-ros-noetic.sh

/opt/Poseidon/install/stages/2-rpi4.sh $hs_if $hs_ssid $hs_pass $wf_if $wf_ssid $wf_pass $snd_ip

/opt/Poseidon/install/stages/3-network.sh

/opt/Poseidon/install/stages/4-rpi.sh

/opt/Poseidon/install/stages/rtc.sh

/opt/Poseidon/install/stages/5-finalize.sh

/opt/Poseidon/install/stages/6-devices-rpi.sh

/opt/Poseidon/install/stages/build-rpi4.sh

echo "*********************************************************************"
echo "*                        Reboot your device                         *"
echo "*********************************************************************"   
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






