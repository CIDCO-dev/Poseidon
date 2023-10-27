#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Set serial number script."
   echo
   echo "Syntax: set_serial_number.sh [options]"
   echo "options:"
   echo "help or h      Print this Help.                  "    
   echo "[serialnumber]   Set serial number          "
   echo
   echo "Command line exemple."
   echo "ethernet-config.sh -help"
   echo "ethernet-config.sh CIDCO-20231012-002 "
   
}

############################################################
# Set serial number                                        #
############################################################
setsn()
{
sudo hostnamectl set-hostname $sn
echo "Serial number and hostname set to :"
hostname
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  sn=$1
  if [ ! -z "$sn" ] 
  then
    setsn
    exit
  else 
    Help
    exit
  fi
fi






