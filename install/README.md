###Install instructions for Raspberry Pi:

Edit UBoot before running the install scripts:

When The RaspBerry Boot Press A Key On Your Keyboard.

Execute The Following Command

U-Boot> setenv bootdelay -2

U-Boot> saveenv

U-Boot> reset

----
###automatic script launch upon new up network interface

1) modify : /etc/network/interfaces
example:
auto eth0
iface eth0 inet dhcp
up ./sync_logfiles.sh

2) put script in : /etc/network/if-up.d

3) give execution right to script 
chmod +x logfiles.sh

---
###rsync passwordless connection
ssh-keygen
ssh-copy-id -i path/key.pub user@server
ssh-add path/private_key
ssh user@server
logout

