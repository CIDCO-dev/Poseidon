#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Create cron job for automatic transfer"
   echo "Be carefull this will erase any other cron job for the user"
   echo
   echo "Parameters to pass:"   
   echo "[user]           				  User Name on server"
   echo "[destination file path]    	  where to store on server"
   echo "[server hostname or ip]    	  where to send files"
   echo "[RSA key filename]            	  SSH key filename stored in ~/.ssh"

   echo
   echo "Command line exemple."
   echo "config_auto_transfer.sh pat test localhost Poseidon"

}

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ] 
	then
    Help
    exit 1
fi

sudo chmod 711 /opt/Poseidon/install/sync_logfiles.sh
echo "@hourly /opt/Poseidon/install/sync_logfiles.sh $1 $2 $3 $4" | sudo tee ubuntu
crontab ubuntu
ssh-keygen -t rsa -N "" -f ~/.ssh/$4
