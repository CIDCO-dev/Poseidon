#!/bin/sh

sudo apt install -y libeigen3-dev libgeographic-dev ros-melodic-robot-localization  | tee log.txt

cd /home/ubuntu/  | tee log.txt
git clone https://github.com/orocos/orocos_kinematics_dynamics.git  | tee log.txt
cd orocos_kinematics_dynamics  
cd orocos_kdl  
cmake .  | tee log.txt
make  | tee log.txt
sudo make install  | tee log.txt
cd ..
cd ..

git clone https://github.com/jbeder/yaml-cpp.git  | tee log.txt
cd yaml-cpp
cmake -DYAML_BUILD_SHARED_LIBS=ON .  | tee log.txt
make  | tee log.txt
sudo make install  | tee log.txt
cd ..
