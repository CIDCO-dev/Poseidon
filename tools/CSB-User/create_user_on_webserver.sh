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
sudo adduser --home $home/ --ingroup $group --disabled-login $uname


echo "Integrate ssh key in the system"
sudo mkdir $home/.ssh/
echo "Add SSH autorisation"
echo "$key" | sudo tee -a $home/.ssh/authorized_keys
sudo chown -R $uname:$group $home/.ssh/ 

sudo mkdir -p $home/dev
cd $home/dev/
sudo mknod -m 666 null c 1 3
sudo mknod -m 666 tty c 5 0
sudo mknod -m 666 zero c 1 5
sudo mknod -m 666 random c 1 8

sudo mkdir -p $home/bin
sudo cp /bin/bash $home/bin/bash


sudo chown root:root $home/
sudo chmod 755 $home/

sudo mkdir -p $home/lib/x86_64-linux-gnu
sudo mkdir -p $home/lib64
sudo cp /lib64/ld-linux-x86-64.so.2 $home/lib64/
sudo cp /lib/x86_64-linux-gnu/{libtinfo.so.6,libdl.so.2,libc.so.6} $home/lib/x86_64-linux-gnu/
sudo mkdir -p $home/etc
sudo cp -f /etc/passwd $home/etc/
sudo cp -f /etc/group $home/etc/


echo "Lock the user in the home directory"
sudo echo | sudo tee -a /etc/ssh/sshd_config
sudo echo "Match User $uname" | sudo tee -a /etc/ssh/sshd_config
sudo echo "	ChrootDirectory $home" | sudo tee -a /etc/ssh/sshd_config
#sudo echo "	ForceCommand internal-sftp" | sudo tee -a /etc/ssh/sshd_config
sudo echo "	AllowTcpForwarding no" | sudo tee -a /etc/ssh/sshd_config
sudo echo "	X11Forwarding no" | sudo tee -a /etc/ssh/sshd_config  
sudo echo | sudo tee -a /etc/ssh/sshd_config


sudo chmod 0700 $home/.ssh/

sudo systemctl restart sshd

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






