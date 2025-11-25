cmake_minimum_required(VERSION 3.5)
project(state_controller_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raspberrypi_vitals_msg REQUIRED)

set(msg_files
  "msg/State.msg"
)

set(srv_files
  "srv/GetStateService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs geometry_msgs raspberrypi_vitals_msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
