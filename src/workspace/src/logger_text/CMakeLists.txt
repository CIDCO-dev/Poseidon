cmake_minimum_required(VERSION 2.8.3)
project(logger_text)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs logger_service)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs logger_service
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
 )
 add_executable(${PROJECT_NAME}_node nodes/${PROJECT_NAME}_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${PROJECT_NAME}
 )
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

if(CATKIN_ENABLE_TESTING)
catkin_add_gtest(test_${PROJECT_NAME} tests/test_${PROJECT_NAME}.cpp)
target_link_libraries(test_${PROJECT_NAME} gnss_dummy  ${logger_text_LIBRARIES})
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

endif()
