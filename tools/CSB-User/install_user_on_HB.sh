#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Download Files From Webserver script."
   echo
   echo "Syntax: install_user_on_HB.sh [options]"
   echo "options:"
   echo "help or h        Print this Help.                  "    
   echo "[name]           CSB User Name"
   echo
   echo "Command line exemple."
   echo "install_user_on_HB.sh -help"
   echo "install_user_on_HB.sh name "
   echo "install_user_on_HB.sh 'CSB1234"
   
}

############################################################
# Command                                                  #
############################################################
Command()
{
ssh-keygen -t rsa -b 4096 -C "$uname@csb.cidco.ca"

echo "SSH key to be implemented on the webserver"
cat /home/ubuntu/.ssh/id_rsa.pub
   
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  uname=$1
 
  if [ ! -z "$uname" ] 
  then
    Command
    exit
  else 
    Help
    exit
  fi
fi






