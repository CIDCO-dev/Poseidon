cmake_minimum_required(VERSION 3.0.2)
project(bilge_pump)

find_package(catkin REQUIRED COMPONENTS rospy)


catkin_package(
  CATKIN_DEPENDS bilge_pump_msg rospy
)

include_directories(include ${catkin_INCLUDE_DIRS})




catkin_install_python(PROGRAMS src/Bilge_Pump.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS listener talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
