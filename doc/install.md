# Installation Guide (RPi4, Ubuntu 20.04, ROS Noetic)

## Prerequisites
- Hardware: Raspberry Pi 4 (4GB+ recommended)
- OS: Ubuntu 20.04 (64-bit)
- Network: Internet access for apt/rosdep
- Privileges: sudo

## One-shot install (recommended)
Use the bundled installer which chains all stage scripts:
```bash
sudo chown $USER:$USER /opt
cd /opt
git clone --recursive <repo-url> Poseidon
cd Poseidon
sh install/rpi4-noetic.sh <hotspot_if> <hotspot_ssid> <hotspot_pass> <wifi_if> <wifi_ssid> <wifi_pass> [rtc]
```
Examples:
- Software RTC: `sh install/rpi4-noetic.sh wlan1 Hydro-B cidco1234 wlan0 test pass-1234`
- Hardware RTC: append `rtc` as last arg.

What it does (in order):
1. `install/stages/1-base-ros-noetic.sh`
2. `install/stages/2-rpi4.sh <hotspot_if> <hotspot_ssid> <hotspot_pass> <wifi_if> <wifi_ssid> <wifi_pass>`
3. `install/stages/3-network.sh`
4. `install/stages/4-rpi.sh`
5. `install/stages/rtc.sh` (if `rtc` arg provided; otherwise still invoked as part of the script)
6. `install/stages/5-finalize.sh`
7. `install/stages/6-devices-rpi.sh`
8. `install/stages/build-rpi4.sh`

At the end, reboot is recommended.

## Manual steps (if you cannot run the installer)
1) Base packages:
```bash
sudo apt-get update
sudo apt-get install -y build-essential ccache cmake git pkg-config curl wget ca-certificates python3 python3-pip python3-venv python3-rosdep
sudo apt-get install -y libssl-dev libboost-system-dev libopencv-dev libpcl-dev libwebsocketpp-dev rapidjson-dev libgps-dev libexiv2-dev libxml2-dev libxslt1-dev libyaml-cpp-dev
```
2) ROS Noetic:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
sudo rosdep init || true
rosdep update
```
3) Clone Poseidon:
```bash
sudo chown $USER:$USER /opt
cd /opt
git clone --recursive <repo-url> Poseidon
cd Poseidon
```
4) Stage scripts (mirroring the installer):
```bash
sh install/stages/1-base-ros-noetic.sh
sh install/stages/2-rpi4.sh <hotspot_if> <hotspot_ssid> <hotspot_pass> <wifi_if> <wifi_ssid> <wifi_pass>
sh install/stages/3-network.sh
sh install/stages/4-rpi.sh
sh install/stages/rtc.sh   # include only if hardware RTC is present
sh install/stages/5-finalize.sh
sh install/stages/6-devices-rpi.sh
sh install/stages/build-rpi4.sh
```
5) Build workspace:
```bash
cd /opt/Poseidon/src/workspace
catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;inertial_sense;velodyne;ins_piksi;velodyne_pointcloud;velodyne_driver;velodyne_laserscan;gnss_mosaic_x5;tf"
source devel/setup.bash
```
6) Run simulator:
```bash
roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch
```
Then browse to `http://localhost`.

7) Tests:
- C++/Python/ROS: `catkin_make run_tests -j1 -DCATKIN_BLACKLIST_PACKAGES="ins_piksi;libmavconn;echoboat_odometry;mavros_msgs;mavros;mavros_extras;test_mavros;gnss_mosaic_x5;imu_bno055;sonar_imagenex852;inertial_sense;raspberrypi_vitals;imu_null;sonar_dummy;gnss_dummy;lidar_filtering"`
- JS: `cd /opt/Poseidon/www/webroot/js && npm test --coverage`

## Notes
- `install/rpi4-noetic.sh` orchestrates all stage scripts; prefer it when possible.
- Keep `doc/` Markdown as the source of truth; PDF generation can be added later via CI.
