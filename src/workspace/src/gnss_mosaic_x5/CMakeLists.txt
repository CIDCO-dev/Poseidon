cmake_minimum_required(VERSION 3.0.2)
project(gnss_mosaic_x5)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
    #src/${PROJECT_NAME}.cpp
#)

#add_executable(${PROJECT_NAME}_node nodes/${PROJECT_NAME}_node.cpp)

#target_link_libraries(${PROJECT_NAME}_node
    #${catkin_LIBRARIES} ${PROJECT_NAME}
#)

#target_link_libraries(${PROJECT_NAME}
    #${catkin_LIBRARIES}
#)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
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
if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
   add_rostest_gtest(test_gnss_mosaic_x5 tests/gnss_mosaic_x5_test.launch tests/test_gnss_mosaic_x5_node.cpp)
   target_link_libraries(test_gnss_mosaic_x5 ${catkin_LIBRARIES})
endif()
