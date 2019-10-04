cmake_minimum_required(VERSION 2.8.3)
project(logger_dummy)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Boost REQUIRED)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(logger_dummy src/main.cpp)
target_link_libraries(logger_dummy ${catkin_LIBRARIES} ${Boost_LIBRARIES})

