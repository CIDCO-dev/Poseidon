cmake_minimum_required(VERSION 3.0.2)
project(ins_piksi)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

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

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}.cpp
# )
#add_executable(${PROJECT_NAME}_node nodes/${PROJECT_NAME}_node.cpp)

#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES} ${PROJECT_NAME}
# )
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
   add_rostest_gtest(test_ins_piksi tests/ins_piksi_test.launch tests/test_ins_piksi_node.cpp)
   target_link_libraries(test_ins_piksi ${catkin_LIBRARIES})
endif()
