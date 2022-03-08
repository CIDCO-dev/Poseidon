#!/bin/bash

if [ $# -eq 0 ]; then
    echo "No key name provided"
    exit 1
fi

sudo chmod 711 /home/ubuntu/Poseidon/install/sync_logfiles.sh
echo "@hourly /home/ubuntu/Poseidon/install/sync_logfiles.sh" | sudo tee ubuntu
crontab ubuntu
ssh-keygen -t rsa -N "" -f $@
