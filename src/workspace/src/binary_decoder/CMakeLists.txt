cmake_minimum_required(VERSION 2.8.3)
project(binary_decoder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(binary_decoder src/main.cpp)
target_link_libraries(binary_decoder ${catkin_LIBRARIES})


