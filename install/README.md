### Install instructions for Raspberry Pi:

Edit UBoot before running the install scripts:

When The RaspBerry Boot Press A Key On Your Keyboard.

Execute The Following Command

U-Boot> setenv bootdelay -2

U-Boot> saveenv

U-Boot> reset

----
### Automatic script launch upon new UP network interface

script example:

```
#!/bin/bash

IF=$1
STATUS=$2

if [ "$IF" == "wlan0" ] && [ "$STATUS" == "up" ];then
    bash /opt/Poseindon/install/sync_logfiles.sh
fi

```
 
- put script in : /etc/NetworkManager/dispatcher.d/

- give execution right to script 
chmod +x script.sh

- sudo systemctl restart network-manager.service && sudo systemctl restart networking.service


---
### rsync passwordless connection
```
ssh-keygen
ssh-copy-id -i path/key.pub user@server
ssh user@server
logout
```
