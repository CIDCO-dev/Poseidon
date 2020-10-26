cmake_minimum_required(VERSION 2.8.3)
project(gpsd_client CXX)

############
## Catkin ##
############

find_package(catkin REQUIRED COMPONENTS
  gps_common
  roscpp
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules (libgps REQUIRED libgps)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${libgps_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/client.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libgps_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # test with launch file
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_gpsd_client_node tests/gpsd_client_node.test tests/test_gpsd_client_node.cpp)
  # target_link_libraries(test_gpsd_client_node ...) #what libraries are needed on rpi?
endif()