cmake_minimum_required(VERSION 3.0.2)
project(diagnostics)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2 
  setting_msg
)
find_package(RapidJSON REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)
find_library(libgps REQUIRED)




# --------Pour C++ --------------  DG
catkin_package(
	INCLUDE_DIRS include
	#LIBRARIES diagnostics
	CATKIN_DEPENDS roscpp std_msgs tf2_geometry_msgs tf2
	DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/diagnostics_websocket.py
  # scripts/api_connection_diagnostic.py
  # scripts/diagnostics_websocket_node.py
  # scripts/binaryStreamGnssDiagnostic.py
  # scripts/clock_diagnostic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


