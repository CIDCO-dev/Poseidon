cmake_minimum_required(VERSION 3.5)
project(logger_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
)

set(srv_files
  "srv/GetLoggingStatus.srv"
  "srv/ToggleLogging.srv"
  "srv/GetLoggingMode.srv"
  "srv/SetLoggingMode.srv"
  "srv/TriggerTransfer.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
