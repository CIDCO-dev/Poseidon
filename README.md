# Poseidon
Hydrographic surveying platform operating system


### Google Cartographer installation

On ROS Noetic:

```
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
mkdir catkin_ws
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
sh src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja
```
