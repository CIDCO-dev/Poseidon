#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Upload Files To Webserver script."
   echo
   echo "Syntax: upload_files_to_webserver.sh [options]"
   echo "options:"
   echo "help or h        Print this Help.                  "    
   echo "[name]           CSB User Name"
   echo "[source]         CSB Source Directoru"
   echo
   echo "Command line exemple."
   echo "upload_files_to_webserver.sh -help"
   echo "upload_files_to_webserver.sh name source"
   echo "upload_files_to_webserver.sh 'csb1234' '/home/ubuntu/Poseidon/www/webroot/record'"
   
}

############################################################
# Command                                                  #
############################################################
Command()
{

rsync -e "ssh" --remove-source-files -z --compress-level=9 -a -H -v --stats $source/*.* $user@csb.cidco.ca:/csb/$user/data
   
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  user=$1
  source=$2
  
  if [ ! -z "$user" ] && [ ! -z "$source" ]
  then
    Command
    exit
  else 
    Help
    exit
  fi
fi






