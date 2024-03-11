#!/bin/bash

wget -q --spider http://google.com

if [ $? -eq 0 ]; then
    echo -e "\e[35m[!] You are connected from the Internet\e[0m"
    echo -e "\e[35m[!] Time Sync on the Internet\e[0m"
    sudo chronyd -q 'pool pool.ntp.org iburst' | tee -a ~/update.log

    echo -e "\e[35m[!] Disabling Unattended Upgrades\e[0m"
    sudo systemctl stop unattended-upgrades | tee -a ~/update.log

    echo -e "\e[35m[!] Update Apt Package\e[0m"
    sudo apt update | tee -a ~/update.log

    echo -e "\e[35m[!] Upgradinf Apt Package\e[0m"
    sudo apt upgrade -y | tee -a ~/update.log

    echo -e "\e[35m[!] Update done \e[0m"
    ail -5 ~/update.log
else
    echo -e "\e[35m[!] You are disconnected from the Internet\e[0m"
fi

