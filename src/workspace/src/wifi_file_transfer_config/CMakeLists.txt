cmake_minimum_required(VERSION 3.0.2)
project(wifi_file_transfer_config)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_runtime
  setting_msg
)


catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES wifi_config
  # CATKIN_DEPENDS rospy std_msgs utils
  # DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../utils/
)

## Installe le node Python
catkin_install_python(
  PROGRAMS scripts/wifi_config_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_wifi_config_node.py)
endif()
