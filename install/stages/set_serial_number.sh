#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   echo "Set serial number script."
   echo
   echo "Syntax: set_serial_number.sh [serialnumber]"
   echo "options:"
   echo "help or h      Print this Help."    
   echo "[serialnumber]   Set serial number as hostname."
   echo
   echo "Command line example:"
   echo "./set_serial_number.sh CIDCO-231012-002"
   echo
   echo "Serial Number Structure:"
   echo "Name-Date-Id"
   echo
   echo "Name = Name (2 to 8 char)"
   echo "Date = Date (format: yyMMdd) (6 digits)"
   echo "Id = Device identification number (3 digits)"
}

############################################################
# Set serial number and hostname                           #
############################################################
setsn()
{
  sn=$1

  echo "Setting hostname to: $sn"
  
  sudo hostnamectl set-hostname "$sn"

  echo "$sn" | sudo tee /etc/hostname >/dev/null

  sudo sed -i "s/^127.0.1.1.*/127.0.1.1 $sn/" /etc/hosts

  sudo systemctl restart systemd-hostnamed

  echo "Hostname successfully set to: $(hostname)"

}

sethotspot()
{
  sn=$1

  echo -e "\e[35m--> Update the SSID\e[0m"

  sudo nmcli con modify Hotspot 802-11-wireless.ssid "$sn"

  echo -e "\e[35m--> Restart the wifi\e[0m"

  sudo nmcli con down Hotspot

  sudo nmcli con up Hotspot
  
}

modifyConfig()
{
  CONFIG_FILE="/opt/Poseidon/config.txt"
  NEW_SSID=$1
  
  if [ -f "$CONFIG_FILE" ]; then
    echo "Modifying hotspotSSID in $CONFIG_FILE..."
    
    # Vérifier si la ligne hotspotSSID existe déjà
    if grep -q "^hotspotSSID " "$CONFIG_FILE"; then
      # Remplacer la ligne existante
      sudo sed -i "s/^hotspotSSID .*/hotspotSSID $NEW_SSID/" "$CONFIG_FILE"
    else
      # Ajouter la ligne à la fin du fichier si elle n'existe pas
      echo "hotspotSSID $NEW_SSID" | sudo tee -a "$CONFIG_FILE" >/dev/null
    fi
    
    echo "hotspotSSID updated successfully to: $NEW_SSID"
  else
    echo "Error: Configuration file $CONFIG_FILE not found!"
  fi
}

# Vérification des arguments
if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]; then
  Help
  exit 1
else
  setsn "$1"
  sethotspot "$1"
  modifyConfig "$1"
  exit 0
fi
