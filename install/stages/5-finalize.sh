#!/bin/bash
echo -e "\[35m[+] install opencv\[0m"
sudo apt-get install libopencv-dev -y | tee -a log.txt

echo -e "\[35m[+] install exiv2\[0m"
sudo apt-get install libexiv2-dev -y | tee -a log.txt

echo -e "\[35m[+] Configuring Inertial Sense SDK\[0m"
cd /opt/Poseidon/src/workspace/src/inertial_sense_ros/
mkdir lib
cd lib
git clone https://github.com/inertialsense/InertialSenseSDK | tee -a log.txt

echo -e "\[35m[+] Install Chrony\[0m"
sudo apt-get install chrony -y | tee -a log.txt
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/../scripts/update-chrony.sh"

echo -e "\[35m[+] Downloading Rtklib\[0m"
cd ~/ 
git clone https://github.com/CIDCO-dev/RTKLIB.git | tee -a log.txt
cd RTKLIB/app
sudo chmod +x makeall.sh
./makeall.sh | tee -a log.txt


