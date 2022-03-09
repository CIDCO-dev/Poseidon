#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Create user on Webserver script."
   echo
   echo "Syntax: create_user_on_webserver.sh [options]"
   echo "options:"
   echo "help or h        Print this Help.                  "    
   echo "[user]           User Name.                        "
   echo "[group]    	  Group                 "
   echo "[home]    	      Home directory                 "
   echo "[key]            SSH key"
 
   echo
   echo "Command line exemple."
   echo "create_user_on_webserver.sh -help"
   echo "create_user_on_webserver.sh user group home key "
   echo "create_user_on_webserver.sh 'CSB1234' 'CSBGroup' '/csb/CSB1234' 'SHA256:QSAD...qwD CSB1234@csb.cidco.ca'"
   
}

############################################################
# Command                                                  #
############################################################
Command()
{
echo "Add user and create home directory"
sudo useradd -m -s /bin/bash -d $home/ -G $group $uname


echo "Integrate ssh key in the system"
sudo mkdir $home/.ssh/
echo "Add SSH autorisation"
echo "$key" | sudo tee -a $home/.ssh/authorized_keys
sudo chown -R $uname:$group $home/.ssh/ 

sudo chmod 0700 $home/.ssh/

sudo mkdir -p $home/data
sudo chown $uname:$group $home/data
}


if [ -z "$1" ] || [ "$1" = 'h' ] || [ "$1" = '-h' ] || [ "$1" = 'help' ] || [ "$1" = '-help' ]
then
  Help
  exit
else 
  uname=$1
  group=$2
  home=$3
  key=$4
  
  if [ ! -z "$uname" ] && [ ! -z "$group" ] && [ ! -z "$home" ] && [ ! -z "$key" ]
  then
    Command
    exit
  else 
    Help
    exit
  fi
fi






