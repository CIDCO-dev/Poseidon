cmake_minimum_required(VERSION 2.8.3)
project(lidar_filtering)

add_definitions(-std=c++17 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_filtering
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library 
add_library(${PROJECT_NAME} src/lidarFilters.cpp)

# executables
add_executable(lidar_filtering_node nodes/lidar_filtering_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(lidar_filtering_node ${catkin_LIBRARIES} ${PROJECT_NAME})


#############
## Install ##
#############


install(TARGETS lidar_filtering_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
    add_rostest_gtest(lidar_filters tests/lidar_filters_tests.launch tests/lidar_filters_tests.cpp)   
    target_link_libraries(lidar_filters ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
