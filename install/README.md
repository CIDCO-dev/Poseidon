Install instructions for Raspberry Pi:

Edit UBoot before running the install scripts:

When The RaspBerry Boot Press A Key On Your Keyboard.

Execute The Following Command

U-Boot> setenv bootdelay -2

U-Boot> saveenv

U-Boot> reset

----
automatic script launch upon new up network interface

1) modify : /etc/network/interfaces
example:
auto wlx64700220065e
iface wlx64700220065e inet dhcp
up ./script.sh

2) put script in : /etc/network/if-up.d

3) give execution right to script 
chmod +x filename.sh
