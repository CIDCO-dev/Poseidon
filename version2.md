# ROS2 Version History (English)

Use this file to record all ROS2-specific work (packages, nodes, builds, migrations). Keep `version.md` for ROS1/legacy updates.

## 2025-11-26
- Updated ROS2 install script to install gpsd/chrony/ros-jazzy-gpsd-client and shared chrony config (PPS, NMEA, pool.ntp.org); lighttpd now points to `/opt/Poseidon/www/webroot`.
- Logger: removed warning when toggling without GPS fix (ROS2).
- CI: GitHub Actions posts coverage summary (C++/Python/JS) to workflow summary.
- CI: Fixed YAML indentation for coverage summary step (workflow parses correctly).
- CI: Fixed heredoc indentation for coverage summary (line 250 error resolved).
- CI: Coverage summary heredoc anchored with indentation (python block).

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
- Added ROS2 package `gnss_zed_f9p` (reads UBX via pyserial when available, publishes speed/gnss_status and binary stream, rotates UBX logs; dummy fallback otherwise).
- Implemented ROS2 `i2c_controller` node (HIH8130 humidity/temp, INA238 voltage/shunt/temp, LED control with logger status timers; mirrors ROS1 actions).
- Upgraded ROS2 `logger` (text and binary nodes) to mirror ROS1 behaviour: mode/speed-based toggling, geofence filtering, LED/i2c integration, GNSS watchdog, log rotation, zipped outputs, and HTTP(S) transfer of zip archives.
- ROS2 `hydroball_config_websocket` now mirrors ROS1: get/save config over WS, get_configuration/zero_imu_offsets services, config topic broadcast, IMU TF offsets, hotspot SSID update.
- ROS2 `hydroball_data_websocket` aligned to ROS1: telemetry JSON (position/attitude/depth/vitals), GNSS status broadcast, logging info on connect/commands, and AlwaysOn auto-start behaviour.
- ROS2 `hydroball_files_websocket` aligned to ROS1: file list/delete, publishfiles workflow with connectivity checks and transfer status via `trigger_transfer`.
- ROS2 `diagnostics` websocket: responds to updateDiagnostic (emits diagnostics list) and getRunningNodes (via `ros2 node list` fallback to rclpy names).
- ROS2 `raspberrypi_vitals` mirrors ROS1 thresholds and LED handling: reads CPU/hdd/load/ram/uptime plus I2C humidity/temperature/voltage/LED, sets warning/critical messages and drives LED states.
- Added ROS2 BNO055 support: upstream `ros-jazzy-bno055` driver installed; local `imu_bno055` package kept as fallback (I2C imu/temp/mag with reset/calibrate services).
- Added ROS2 `imu_dummy` node: publishes oscillating IMU orientations on `imu/data` at 200 Hz.
- Added ROS2 `imu_null` node: publishes zeroed IMU orientation on `imu/data` at 200 Hz.
- Added ROS2 `power_control` node: drives GPIO LEDs/pulse/shutdown pin via `led_control` messages (simulation fallback if RPi.GPIO missing).
- Added ROS2 `power_management` node: monitors voltage via i2c_controller_service and triggers `systemctl stop ros` below threshold.
- Added ROS2 `sonar_dummy` node: publishes oscillating depth on `depth` and reads sonar config keys from `get_configuration` service.
- Added ROS2 `sonar_imagenex852_c` node: serial driver for Imagenex 852 (auto/manual trigger, config updates, depth/ENU, binary stream).
- Added ROS2 `sonar_nmea_0183_tcp_client` node: NMEA TCP client parsing GGA/DBT/DPT/ADS/VTG to publish `depth`, `fix`, and `speed`.
- Added ROS2 `wifi_file_transfer_config` node: manages WiFi connections via `nmcli` based on config keys (SSID/password/autoconnect).
- Updated ROS2 install script to install gpsd/chrony/ros-jazzy-gpsd-client and shared chrony config (PPS, NMEA, pool.ntp.org); lighttpd now points to `/opt/Poseidon/www/webroot`.
