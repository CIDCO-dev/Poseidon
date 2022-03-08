#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Download Files From Webserver script."
   echo
   echo "Syntax: download_files_from_webserver.sh [options]"
   echo "options:"
   echo "help or h        Print this Help.                  "    
   echo "[webserver]      Webserver address.                "
   echo "[destination]    File Destination.                 "
   echo
   echo "Command line exemple."
   echo "download_files_from_webserver.sh -help"
   echo "download_files_from_webserver.sh webserver destination "
   echo "download_files_from_webserver.sh 'csb.cidco.ca' '/home/csb/'"
   
}

############################################################
# Command                                                  #
############################################################
Command()
{

rsync -az -P --exclude 'bin' --exclude 'etc' --exclude 'dev' --exclude 'lib' --exclude 'lib64' --exclude '.ssh' --exclude '.profile' --exclude '.bashrc' --exclude 'bash_logout' --remove-source-files csbclient@csb.cidco.ca/csb/ /data/csb

   
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  webadd=$1
  dest=$2
  
  if [ ! -z "$webadd" ] && [ ! -z "$dest" ]
  then
    Command
    exit
  else 
    Help
    exit
  fi
fi






