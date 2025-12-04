cmake_minimum_required(VERSION 2.8.3)
project(logger)

add_definitions(-std=c++17 -Wall -g)

find_package(catkin REQUIRED COMPONENTS 
roscpp 
std_msgs 
geometry_msgs 
logger_service 
tf2 
tf2_ros 
tf2_geometry_msgs
binary_stream_msg
i2c_controller_service
setting_msg)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Threads )
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES logger
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs logger_service tf2 tf2_ros tf2_geometry_msgs lidar_filtering setting_msg
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ../lidar_filtering/include
)

## Declare a C++ library 
add_library(${PROJECT_NAME} src/loggerBase.cpp 
							src/loggerText.cpp 
							src/loggerBinary.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(sbet_decoder src/SbetProcessor.cpp)

# executables
add_executable(logger_text_node nodes/logger_text_node.cpp)
add_executable(logger_binary_node nodes/logger_binary_node.cpp)
add_executable(poseidonReader src/poseidon_reader.cpp)
add_executable(lidarGeoreferencer src/georefLog.cpp)
add_executable(calib_printer src/calib_lidar_printer.cpp)
add_executable(vital_printer src/vitals_printer.cpp)
add_dependencies(logger_text_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(logger_binary_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(poseidonReader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidarGeoreferencer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(calib_printer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vital_printer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(logger_text_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} Boost::system OpenSSL::SSL OpenSSL::Crypto)
target_link_libraries(logger_binary_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} Boost::system OpenSSL::SSL OpenSSL::Crypto)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} Boost::system OpenSSL::SSL OpenSSL::Crypto)
target_link_libraries(lidarGeoreferencer Eigen3::Eigen sbet_decoder)
target_link_libraries(calib_printer sbet_decoder)


#############
## Install ##
#############


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
    add_rostest_gtest(test_logger_text_node tests/logger_text_node_test.launch tests/test_logger_text_node.cpp)
    target_link_libraries(test_logger_text_node ${PROJECT_NAME} Boost::system OpenSSL::SSL OpenSSL::Crypto)
    
    add_rostest_gtest(test_logger_binary_node tests/logger_binary_node_test.launch tests/test_logger_binary_node.cpp)
    target_link_libraries(test_logger_binary_node ${PROJECT_NAME} Boost::system OpenSSL::SSL OpenSSL::Crypto)
    
endif()
