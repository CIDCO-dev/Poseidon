# Poseidon
Hydrographic surveying platform operating system

NOTE: 
Clone the repo in /opt for proper use.
call the installation file located in install/ acording to the environement you have.

### Google Cartographer installation

On ROS Noetic:

```
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
mkdir catkin_ws
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```
according to : https://github.com/cartographer-project/cartographer_ros/issues/1726
delete line 46 or the line containing`<depend>libabsl-dev</depend>`
```
sed -i '46d' filename
```
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
sh src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja
```

### Compile and run Poseidon on virtual machine

Pre-requisite: \
Ubuntu virtual machine (ubuntu only) \
The username must be ubuntu ( optional but easier install) \
Instruction:
```
sudo apt install git
cd
git clone --recursive <poseidon repo>
```
Run these scripts:
- ~/Poseidon/install/stages/1-base-ros-noetic.sh
- ~/Poseidon/install/stages/2-x64.sh
- ~/Poseidon/install/stages/3-network.sh
- ~/Poseidon/install/stages/4-x64.sh
- ~/Poseidon/install/stages/5-finalize.sh

Example :
```
sh ~/Poseidon/install/stages/1-base-ros-noetic.sh
```
Source ros environement:
```
source /opt/ros/noetic/setup.bash
```

Optional, automatic sourcing ros environement:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
Start new terminal or :
```
source ~/.bashrc
```

Changes to build poseidon on virtual machine:
```
cd ~/Poseidon/src/workspace
catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;mavros_extras;mavros_msgs;test_mavros;libmavconn;raspberrypi_vitals"
```
Source poseidon workspace:
```
source devel/setup.bash
```
Or
```
echo "source ~/Poseidon/src/workspace/devel/setup.bash" >> ~/.bashrc
```
Start new terminal or :
```
source ~/.bashrc
```

Launch simulator:
```
roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch
```

Start browser and navigate to localhost or 127.0.0.1

### Unit test

See readme for tests :
- sonar_nmea_0183_tcp_client
- gnss_zed_f9p


Run tests
```
catkin_make run_tests -j1 -DCATKIN_BLACKLIST_PACKAGES="ins_piksi;libmavconn;echoboat_odometry;mavros_msgs;mavros;mavros_extras;test_mavros;gnss_mosaic_x5;imu_bno055;sonar_imagenex852;inertial_sense;raspberrypi_vitals;imu_null;sonar_dummy;gnss_dummy"

```

### Replay Echoboat Rosbag
In poseidon workspace:
```
roslaunch launch/Echoboat/rosbag_replay.launch bag_filename:=rosbag.bag
```

### Geo-reference Lidar
In poseidon workspace:
```
./devel/lib/logger/lidarGeoreferencer file > outputFile.txt
```
Activate filtering:
```
./devel/lib/logger/lidarGeoreferencer -f file > outputFile.txt
```
For help:
```
./devel/lib/logger/lidarGeoreferencer
```


### Logging mode

- Always ON : 1
- Manual : 2
- Speed based : 3


