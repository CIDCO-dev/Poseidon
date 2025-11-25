# ROS2 Version History (English)

Use this file to record all ROS2-specific work (packages, nodes, builds, migrations). Keep `version.md` for ROS1/legacy updates.

## 2025-11-25
- Created ROS2 workspace scaffold `src/ros2_ws` with Python `logger` package (stub) exposing the logging service API (toggle/get/set mode/status, trigger transfer).
- Added ROS2 interface package `logger_interfaces` carrying the logger service definitions for reuse by other ROS2 nodes.
- Renamed ROS2 logger package to `logger` and provided entry points for `logger_node`, `logger_binary_node`, and `logger_text_node` with preserved names.
- Added ROS2 interface packages `raspberrypi_vitals_msg` and `state_controller_msg` (ported message/service definitions).
- Added ROS2 Python port `state_controller` (node name `stateControl`) mirroring ROS1 topics/services (`state` publisher and `get_state` service).
