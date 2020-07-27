#!/bin/sh

sudo apt install -y libeigen3-dev libgeographic-dev ros-melodic-robot-localization

cd /home/ubuntu/
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
cd orocos_kdl
cmake .
make
sudo make install
cd ..
cd ..

git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
cmake -DYAML_BUILD_SHARED_LIBS=ON .
make
sudo make install
cd ..
