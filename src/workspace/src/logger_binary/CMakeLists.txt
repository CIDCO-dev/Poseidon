cmake_minimum_required(VERSION 2.8.3)
project(logger_binary)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(logger_binary src/main.cpp)
target_link_libraries(logger_binary ${catkin_LIBRARIES})

