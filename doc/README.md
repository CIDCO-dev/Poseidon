# Poseidon Documentation

## Overview
- ROS Noetic hydrographic platform.
- Catkin workspace: `src/workspace`.
- Web UI: `www/webroot`.
- Install scripts: `install/stages/`.

## Prerequisites
- Ubuntu 20.04 with ROS Noetic (`/opt/ros/noetic`).
- Build tools: gcc/g++, cmake, ccache, python3, pip, rosdep.
- Node.js 18+ for JS tests (CI uses `actions/setup-node@v4`).

## Installation (RPi4, Ubuntu 20.04, ROS Noetic)
```bash
sudo chown $USER:$USER /opt
cd /opt
git clone --recursive <repo>
sh /opt/Poseidon/install/rpi4-noetic.sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/Poseidon/src/workspace/devel/setup.bash" >> ~/.bashrc
```

## Build
```bash
cd /opt/Poseidon/src/workspace
catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;inertial_sense;velodyne;ins_piksi;velodyne_pointcloud;velodyne_driver;velodyne_laserscan;gnss_mosaic_x5;tf"
source devel/setup.bash
```

## Run (Simulator)
```bash
roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch
```
Open the web UI at `http://localhost`.

## Tests
- ROS/C++: `catkin_make run_tests -j1 -DCATKIN_BLACKLIST_PACKAGES="ins_piksi;libmavconn;echoboat_odometry;mavros_msgs;mavros;mavros_extras;test_mavros;gnss_mosaic_x5;imu_bno055;sonar_imagenex852;inertial_sense;raspberrypi_vitals;imu_null;sonar_dummy;gnss_dummy;lidar_filtering"`
- JS: `cd www/webroot/js && npm test --coverage`
- Coverage summary: produced in CI (C++/Python/JS).

## Tools & Operations
- Rosbag replay: `roslaunch launch/Echoboat/rosbag_replay.launch bag_filename:=rosbag.bag`
- Lidar georef: `./devel/lib/logger/lidarGeoreferencer -h` (supports filtering with `-f -a -b -d -e`)
- Logging modes: 1=Always ON, 2=Manual, 3=Speed based

## Structure
- `src/workspace/src/<package>`: ROS packages
- `src/workspace/launch/`: launch files
- `www/webroot`: web UI assets and JS
- `install/stages`: install scripts
- `config.txt`: runtime config (no real secrets)

## Notes
- Keep docs up to date here; CI can generate PDFs later if needed.
- For embedded targets, adjust blacklist/launch files for resources.
