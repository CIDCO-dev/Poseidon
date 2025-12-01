# Poseidon
Hydrographic surveying platform operating system (ROS Noetic).

## Getting started (RPi4, Ubuntu 20.04, ROS Noetic)
- Clone under `/opt`: `sudo chown $USER:$USER /opt && cd /opt && git clone --recursive <repo>`
- Install: run `install/rpi4-noetic.sh` (RPi4 target).
- Build: `cd /opt/Poseidon/src/workspace && catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;inertial_sense;velodyne;ins_piksi;velodyne_pointcloud;velodyne_driver;velodyne_laserscan;gnss_mosaic_x5;tf"`
- Source: `source /opt/ros/noetic/setup.bash` then `source /opt/Poseidon/src/workspace/devel/setup.bash`.
- Launch simulator: `roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch`, then open the web UI at `http://localhost`.

## Docs
Full documentation lives in `doc/` (Markdown):
- `doc/README.md`: overview, build, run, tests
- `doc/install.md`: installation with dependencies (RPi4, Ubuntu 20.04, ROS Noetic)
- `doc/nodes.md`: list of ROS nodes
- `doc/web.md`: list of web UI pages
- `doc/web/*.md`: per-page details (index, status, diagnostics, data, map, settings, calibration, goal)
- `doc/nodes/*.md`: per-node details with inputs/outputs/params

## Tests
- C++/Python/ROS: `catkin_make run_tests -j1 -DCATKIN_BLACKLIST_PACKAGES="ins_piksi;libmavconn;echoboat_odometry;mavros_msgs;mavros;mavros_extras;test_mavros;gnss_mosaic_x5;imu_bno055;sonar_imagenex852;inertial_sense;raspberrypi_vitals;imu_null;sonar_dummy;gnss_dummy;lidar_filtering"`
- JS: `cd www/webroot/js && npm test --coverage`

## Tools
- Rosbag replay: `roslaunch launch/Echoboat/rosbag_replay.launch bag_filename:=rosbag.bag`
- Lidar georef: `./devel/lib/logger/lidarGeoreferencer -h` (supports filtering options)
- Logging modes: 1=Always ON, 2=Manual, 3=Speed based
