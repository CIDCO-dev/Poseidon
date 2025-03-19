#!/bin/bash

echo "[+] Downloading WiringPi"
cd ~/ 
git clone https://github.com/CIDCO-dev/WiringPi.git | tee -a log.txt
cd WiringPi/
./build | tee -a log.txt

echo "[+] Configuring UART"
sudo bash -c 'echo "dtoverlay=disable-bt" >> /boot/firmware/usercfg.txt' | tee -a log.txt
sudo systemctl mask serial-getty@ttyAMA0.service | tee -a log.txt
usermod -G dialout ubuntu

echo "[+] Install GPSD"
sudo apt-get install gpsd gpsd-clients libgps-dev -y | tee -a log.txt
sudo cp /etc/default/gpsd "/etc/default/gpsd.bak$(date +"%Y%m%d_%H%M%S")"

sudo bash -c 'cat << EOF2 > /etc/default/gpsd
# Default settings for the gpsd init script and the hotplug wrapper.
# Start the gpsd daemon automatically at boot time
START_DAEMON="true"
# Use USB hotplugging to add new USB devices automatically to the daemon
USBAUTO="true"
# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="/dev/ttyAMA0"
# Other options you want to pass to gpsd
GPSD_OPTIONS="-n"
GPSD_SOCKET="/var/run/gpsd.sock"
EOF2'

sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/

echo "[+] Install PPS"
sudo apt-get install pps-tools -y | tee -a log.txt

sudo bash -c 'echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/usercfg.txt'  
sudo bash -c 'echo "dtoverlay=uart0" >> /boot/firmware/usercfg.txt'
sudo bash -c 'echo "pps-ldisc" >> /etc/modules'

echo "[+] Config I2C"
sudo bash -c 'echo "dtparam=i2c_arm_baudrate=500000" >> /boot/firmware/usercfg.txt'
sudo bash -c 'echo "dtparam=i2c4,pins_6_7" >> /boot/firmware/usercfg.txt'

echo "[+] Turn off SPI"
sudo sed -i "s/dtparam=spi=on/dtparam=spi=off/g" /boot/firmware/syscfg.txt

echo "[+] Config uart"
sudo bash -c 'cat << EOF2 > /boot/firmware/cmdline.txt
net.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc
EOF2'



echo "[+] Adding Hardware RTC on boot"
sudo bash -c 'cat << EOF4 > /etc/systemd/system/hwrtc.service
[Unit]
Description=Synchronise System clock to hardware RTC
DefaultDependencies=no
After=systemd-modules-load.service
Before=systemd-journald.service time-sync.target sysinit.target shutdown.target
Conflicts=shutdown.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/hwclock --hctosys --utc --noadjfile
RestrictRealtime=yes

[Install]
WantedBy=sysinit.target
EOF4'
sudo chmod 755 /etc/systemd/system/hwrtc.service
sudo systemctl enable hwrtc

echo "[+] Adding roslaunch on boot"
sudo bash -c 'cat << EOF3 > /etc/systemd/system/ros.service
[Unit]
Description=Launch ROS on boot.
After=gpsd.service hwrtc.service

[Service]
Type=simple
KillSignal=SIGINT
ExecStart=/opt/Poseidon/launchROSService.sh
ExecStop=/usr/bin/pkill -INT roslaunch
TimeoutStopSec=60
ExecStopPost=/sbin/shutdown now
[Install]
WantedBy=multi-user.target
EOF3'

sudo chmod 755 /etc/systemd/system/ros.service
sudo systemctl enable ros
echo "sudo systemctl start ros"

echo "[+] Adding Uart config on boot"
sudo bash -c 'cat << EOF5 > /etc/systemd/system/uart.service
[Unit]
Description=Launch Uart config on boot.
Before=gpsd.service

[Service]
Type=simple
ExecStart=/opt/Poseidon/service/uart_on_boot.sh

[Install]
WantedBy=multi-user.target
EOF5'

sudo chmod 755 /etc/systemd/system/uart.service
sudo systemctl enable uart


