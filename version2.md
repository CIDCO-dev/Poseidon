# ROS2 Version History (English)

Use this file to record all ROS2-specific work (packages, nodes, builds, migrations). Keep `version.md` for ROS1/legacy updates.

## 2025-11-25
- Created ROS2 workspace scaffold `src/ros2_ws` with Python `logger` package (stub) exposing the logging service API (toggle/get/set mode/status, trigger transfer).
- Added ROS2 interface package `logger_interfaces` carrying the logger service definitions for reuse by other ROS2 nodes.
- Renamed ROS2 logger package to `logger` and provided entry points for `logger_node`, `logger_binary_node`, and `logger_text_node` with preserved names.
- Added ROS2 interface packages `raspberrypi_vitals_msg` and `state_controller_msg` (ported message/service definitions).
- Added ROS2 interface packages `setting_msg` and `gnss_status_msg` (ported message/service definitions).
- Added ROS2 Python port `state_controller` (node name `stateControl`) mirroring ROS1 topics/services (`state` publisher and `get_state` service).
- Added ROS2 Python package `hydroball_ws` with stub nodes for `hydroball_config_websocket`, `hydroball_data_websocket`, `hydroball_files_websocket`, and `diagnostics` (names preserved).
- Implemented ROS2 `hydroball_data_websocket` (WebSocket server, telemetry broadcast, logger control) using `websockets`.
- Implemented ROS2 `hydroball_config_websocket` (config load/save, zero IMU offsets, config topic, static IMU TF) using `websockets`.
- Implemented ROS2 `hydroball_files_websocket` (file list/delete, transfer trigger/status with connectivity checks) using `websockets`.
- Implemented ROS2 `diagnostics` WebSocket (responds to `updateDiagnostic`/`getRunningNodes`) using `websockets`.
- Added ROS2 packages `i2c_controller_service` (srv) and `raspberrypi_vitals` (publishes vitals, calls i2c controller) ported from ROS1.
- Added ROS2 package `gnss_dummy` (dummy GNSS publisher).
- Added ROS2 package `gnss_zed_f9p` (stub publisher for speed and gnss_status).
