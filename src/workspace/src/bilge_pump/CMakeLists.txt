cmake_minimum_required(VERSION 3.0.2)
project(bilge_pump)

find_package(catkin REQUIRED COMPONENTS rospy)

#catkin_python_setup() needed if there is need to  import python modules
	#needs to call setup.py file

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy

)

include_directories(include ${catkin_INCLUDE_DIRS})




catkin_install_python(PROGRAMS src/bilge_pump.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

