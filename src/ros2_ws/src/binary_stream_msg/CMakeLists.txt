cmake_minimum_required(VERSION 3.5)
project(binary_stream_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Stream.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
