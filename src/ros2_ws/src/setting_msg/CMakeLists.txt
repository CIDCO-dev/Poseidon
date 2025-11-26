cmake_minimum_required(VERSION 3.5)
project(setting_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Setting.msg"
)

set(srv_files
  "srv/ConfigurationService.srv"
  "srv/ImuOffsetService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
