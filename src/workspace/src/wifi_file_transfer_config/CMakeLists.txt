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

## Declare a C++ library
<<<<<<< HEAD
# add_library(${PROJECT_NAME} src/wifi_config.cpp)
=======
add_library(${PROJECT_NAME} src/wifi_config.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/wifi_config_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${PROJECT_NAME}
 )
 target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
>>>>>>> 7e7e8ebb (add dependencies)


