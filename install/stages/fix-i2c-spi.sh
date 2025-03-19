#!/bin/sh


echo "[+] Config I2C"
sudo bash -c 'echo "dtparam=i2c4,pins_6_7" >> /boot/firmware/usercfg.txt'

echo "[+] Turn off SPI"
sudo sed -i "s/dtparam=spi=on/dtparam=spi=off/g" /boot/firmware/syscfg.txt
