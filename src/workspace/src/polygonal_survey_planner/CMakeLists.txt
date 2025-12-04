cmake_minimum_required(VERSION 3.0.2)
project(polygonal_survey_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
