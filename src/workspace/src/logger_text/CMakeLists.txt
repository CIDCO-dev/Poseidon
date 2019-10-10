cmake_minimum_required(VERSION 2.8.3)
project(logger_text)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(logger_text src/main.cpp)
target_link_libraries(logger_text ${catkin_LIBRARIES})
