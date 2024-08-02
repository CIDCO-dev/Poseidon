### Install instructions for Raspberry Pi:

Edit UBoot before running the install scripts:

When The RaspBerry Boot Press A Key On Your Keyboard.

Execute The Following Command

U-Boot> setenv bootdelay -2

U-Boot> saveenv

U-Boot> reset

----

execute the rscript acording to the material and os version

ubuntu 18 = melodic
ubuntu 20 = noetic

Rpi4 with ubuntu 20 server use rpi4-noetic.sh

### Automatic script launch upon new UP network interface

script example:

```
#!/bin/bash

IF=$1
STATUS=$2

if [ "$IF" == "wlan0" ]
then
    case "$2" in
        up)
        sh /opt/Poseidon/tools/CSB-User/upload_files_to_webserver.sh USER Directory2Transfer
        ;;
        down)
        #command 
        ;;
        *)
        ;;
    esac
fi


```
 
- put script in : /etc/NetworkManager/dispatcher.d/

- give execution right to script 
chmod +x script.sh

- sudo systemctl restart network-manager.service && sudo systemctl restart networking.service

- The RSA key for rsync connection needs be in root directory since dispatcher script are ran by root


---
### rsync passwordless connection
```
ssh-keygen
ssh-copy-id -i path/key.pub user@server
ssh user@server
logout
```
