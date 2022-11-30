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
