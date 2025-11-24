cmake_minimum_required(VERSION 2.8.3)
project(video_recorder)

add_definitions(-std=c++17 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )
find_package(OpenCV 4 REQUIRED)
find_package(REQUIRED CONFIG NAMES exiv2)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
#[[
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
 )
]]
 add_executable(${PROJECT_NAME}_node nodes/${PROJECT_NAME}_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS} exiv2
   # ${PROJECT_NAME}
 )
 #[[
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
 ]]

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############
#[[
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_logger_text_node tests/logger_text_node_test.launch tests/test_logger_text_node.cpp)
    target_link_libraries(test_logger_text_node ${PROJECT_NAME})
endif()
]]
