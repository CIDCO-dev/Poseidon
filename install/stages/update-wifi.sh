#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Script used to update the wifi connection."
   echo
   echo "Syntax: update-wifi.sh [options]"
   echo "options:"
   echo "help or h      Print this Help.                  "    
   echo "[wifi_if]        Wifi interface name.              $wf_if"
   echo "[wifi_ssid]      Wifi SSID.                        $wf_ssid"
   echo "[wifi_pass]      Wifi Password.                    $wf_pass"
   echo
   echo "Command line exemple."
   echo "update-wifi.sh -help"
   echo "update-wifi.sh wifi_if wifi_ssid wifi_pass"
   echo "update-wifi.sh 'wlan0' 'test' 'pass-test' 
   
}

############################################################
# Cfg ethernet                                             #
############################################################
Config()
{
echo "Wifi interface name.              $wf_if"
echo "Wifi SSID.                        $wf_ssid"
echo "Wifi Password.                    $wf_pass"

echo "Creating Wifi connection"
sudo nmcli dev wifi connect $wf_ssid password $wf_pass ifname $wf_if

echo "Apply wifi configuration"
sudo nmcli con reload
sudo service network-manager restart

  
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  wf_if=$1
  wf_ssid=$2
  wf_pass=$3
  if [ ! -z "$wf_if" ] && [ ! -z "$wf_ssid" ] && [ ! -z "$wf_pass" ]
  then
    Config
    exit
  else 
    Help
    exit
  fi
fi






