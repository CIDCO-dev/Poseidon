1) extract these topics:

- /lidar
- /imu
- /gps

python3 tools/Edit-Rosbag/topic_extracter.py inputBag.bag outputBag.bag /imu /lidar /gps
```
python3 tools/Edit-Rosbag/topic_extracter.py ~/bigBag.bag ~/bag_4_cartographer.bag /velodyne_points /imu/data /fix
```

2) move lua file to cartographer workspace:
mv Poseidon/src/workspace/launch/Cartographer/echoboat2D.lua cartographer_workspace/install_isolated/share/cartographer_ros/configuration_files/echoboat2D.lua

3) move both launch files to cartographer workspace:
mv Poseidon/src/workspace/launch/Cartographer/demo_echoboat.launch cartographer_workspace/install_isolated/share/cartographer_ros/launch/demo_echoboat.launch
mv Poseidon/src/workspace/launch/Cartographer/echoboat.launch cartographer_workspace/install_isolated/share/cartographer_ros/launch/echoboat.launch


4) launch
source install_isolated/setup.bash
roslaunch cartographer_ros demo_echoboat.launch bag_filename:=/home/ubuntu/bag_4_cartographer.bag

cartographer gives error : 
F1128 14:37:07.504280 18835 sensor_bridge.cc:140] Check failed: msg->linear_acceleration_covariance[0] != -1 (-1 vs. -1) Your IMU data claims to not contain linear acceleration measurements by setting linear_acceleration_covariance[0] to -1. Cartographer requires this data to work. 

error is comming from ros_sbg_driver line 1031:
https://github.com/SBG-Systems/sbg_ros_driver/blob/ce60bb858dd19eb7a72913347c983a4d2d4ccbba/src/message_wrapper.cpp

