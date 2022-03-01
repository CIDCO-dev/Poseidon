# Poseidon
Hydrographic surveying platform operating system


### Google Cartographer installation

On ROS Noetic:

```
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
mkdir catkin_ws
cd catkin_ws
wstool init src<
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
sh src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja
```
### Logging mode
```
always ON : 1
Manual : 2
Speed based : 3
```

### Compile and run Poseidon on virtual machine
```
pre-requisite:
ubuntu (ubuntu only) virtual machine
 the username must be ubuntu ( optional but easier install)
 instruction:
>sudo apt install git
>cd
>git clone <poseidon repo>
run these scripts:
	~/Poseidon/install/stages/1-base-ros-noetic.sh
	~/Poseidon/install/stages/2-x64.sh
	~/Poseidon/install/stages/3-network.sh
	~/Poseidon/install/stages/4-x64.sh
	~/Poseidon/install/stages/5-finalize.sh

example :
>sh ~/Poseidon/install/stages/1-base-ros-noetic.sh

source ros environement:
>source /opt/ros/noetic/setup.bash

optional, automatic sourcing ros environement:
>echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
start new terminal or :
>source ~/.bashrc


changes to build poseidon on virtual machine:
>cd ~/Poseidon/src/workspace
>catkin_make -DCATKIN_BLACKLIST_PACKAGES="mavros;mavros_extras;mavros_msgs;test_mavros;libmavconn;echoboat_odometry;raspberrypi_vitals"

build:
>catkin_make
source poseidon workspace:
>source devel/setup.bash
or
>echo "source ~/Poseidon/src/workspace/devel/setup.bash" >> ~/.bashrc
start new terminal or :
>source ~/.bashrc

launch test environement:
>python3 ~/Poseidon/src/workspace/src/sonar_nmea_0183_tcp_client/src/send_data.py
>roslaunch launch/dummy_simulator_virtual-machine.launch

start browser and navigate to localhost or 127.0.0.1
```
### unit test
some of these tests needs to be repaired, some requires the physical equipement

```
catkin_make run_tests -DCATKIN_BLACKLIST_PACKAGES="libmavconn;echoboat_odometry;mavros_msgs;mavros;mavros_extras;test_mavros;gnss_mosaic_x5;
imu_bno055;sonar_imagenex852;inertial_sense;raspberrypi_vitals;imu_null;sonar_dummy;gnss_dummy"

```

tests that needs to be investigated because they are failing:
```
tf/tests/tf_unittest.cpp
setting_msg/tests/test_zero_imu_offset_service.cpp
ins_piksi
setting_msg/tests/test_get_configuration.cpp
```

node without test:
```
hydroball_files_websocket
hydroball_config_websocket
raspberry_vitals_msg
gps_common
gpsd_client
logger_binary
state_controller_msg
```
