#[[
list_lidar_problematic_message_id

written by : Patrick Charron-Morneau
Date : 23th nov 2022

]]
cmake_minimum_required(VERSION 3.10)

# set the project name
project(llpmi VERSION 0.0.01)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

INCLUDE_DIRECTORIES(/opt/ros/noetic/include)

find_package(rosbag REQUIRED PATHS /opt/ros/noetic/share/rosbag/cmake/)
include_directories(${rosbag_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} list_lidar_problematic_message_id.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${rosbag_LIBRARIES})


