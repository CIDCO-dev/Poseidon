cmake_minimum_required(VERSION 2.8.3)
project(logger)

add_definitions(-std=c++17 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs logger_service tf2 tf2_ros tf2_geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES logger
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs logger_service tf2 tf2_ros tf2_geometry_msgs
  DEPENDS system_lib
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

## Declare a C++ library
 
 add_library(${PROJECT_NAME}
   src/loggerBase.cpp src/loggerText.cpp
 )


add_executable(logger_text_node nodes/logger_text_node.cpp)
#add_executable(logger_binary_node nodes/logger_binary_node.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(logger_text_node
   ${catkin_LIBRARIES} ${PROJECT_NAME}
 )

# target_link_libraries(logger_binary_node
#   ${catkin_LIBRARIES} ${PROJECT_NAME}
# )

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )



#############
## Install ##
#############


 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)
#find_package(rostest REQUIRED)
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )
#add_rostest_gtest(test_sonar_nmea_0183_device_client tests/sonar_nmea_0183_test.launch tests/test_sonar_nmea_0183_device_client.cpp)
#target_link_libraries(test_sonar_nmea_0183_device_client sonar_nmea_0183_tcp_client ${sonar_nmea_0183_tcp_client_LIBRARIES})
#add_dependencies(test_sonar_nmea_0183_device_client ${catkin_EXPORTED_TARGETS}) #? ${catkin_LIBRARIES}

#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
