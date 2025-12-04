cmake_minimum_required(VERSION 2.8.3)
project(inertial_sense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  message_generation
  tf
)
find_package(Threads)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

add_message_files(
  FILES
  GTime.msg
  SatInfo.msg
  GPS.msg
  GPSInfo.msg
  PreIntIMU.msg
  RTKInfo.msg
  RTKRel.msg
  GlonassEphemeris.msg
  GNSSEphemeris.msg
  GNSSObservation.msg
  GNSSObsVec.msg
)

add_service_files(
  FILES
  FirmwareUpdate.srv
  refLLAUpdate.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES inertial_sense_ros
    CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  library/InertialSenseSDK/src #This line of CMakeList.txt stays in .external file to reference submodule
)

add_subdirectory(library/InertialSenseSDK)

add_library(inertial_sense_ros
        src/inertial_sense.cpp
)
target_link_libraries(inertial_sense_ros InertialSense ${catkin_LIBRARIES} pthread)
target_include_directories(inertial_sense_ros PUBLIC include library/InertialSenseSDK/src)
add_dependencies(inertial_sense_ros inertial_sense_generate_messages_cpp)

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros ${catkin_LIBRARIES})

install(TARGETS inertial_sense_ros inertial_sense_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


if(CATKIN_ENABLE_TESTING)
  # test with launch file
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_inertialsense_node tests/inertialsense_node.test tests/test_inertialsense_node.cpp)
  target_link_libraries(test_inertialsense_node inertial_sense_ros ${catkin_LIBRARIES})
endif()
