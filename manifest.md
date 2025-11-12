# Poseidon — Project Manifest

This document summarizes Poseidon: goals, architecture, dependencies, install/build/run procedures, key launch profiles, and core ROS nodes and topics.

## Overview
- Platform: ROS Noetic (Ubuntu 20.04), Catkin workspace.
- Purpose: Embedded OS for hydrographic surveying (GNSS, IMU, sonar, lidar, power management, logging, on‑device web UI).
- License: MIT (see `LICENSE`).

## Top‑Level Layout
- `src/workspace`: Catkin workspace
  - `src`: ROS packages (C++/Python): `logger`, `gnss_zed_f9p`, `imu_bno055`, `sonar_*`, `state_controller`, `hydroball_*_websocket`, `power_*`, etc.
  - `launch`: `roslaunch` files (Echoboat, Hydrobox, Hydroball, Simulator, Cartographer, Rosbag, Binairy)
  - `config`: workspace‑specific config
- `install`: install/provision scripts (stages for x64, RPi, RockPi)
- `tools`: utilities (rosbag editing, lidar georeferencing, GNSS config, I2C tools)
- `www/webroot`: static web UI (dashboard, data, settings, map)
- `service`: system scripts (e.g., `uart_on_boot.sh`)
- `georef`: Python library for geodesy/ENU + tests
- `README.md`: install/build/run (VM and simulation)
- `Jenkinsfile`: CI pipeline (unit tests on ROS VM)

## Launch Entry Points
- WebSockets only: `roslaunch launch/websocket.launch`
- VM simulator: `roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch`
- Echoboat (example): `roslaunch launch/Echoboat/echoboat.launch`
- Hydrobox (RPi, NMEA sonar + ZED‑F9P + BNO055): `roslaunch launch/Hydrobox/hydrobox_rpi_nmeadevice_ZED-F9P_bno055.launch`
- GNSS helpers: scripts `bin_mosaic-x5.sh`, `bin_piksi.sh`

`launchROSService.sh` centralizes common launch profiles and is used by the systemd `ros` service.

## Configurations & Features

### Common Features
- GNSS via gpsd + vendor node: publishes `fix`, `speed`, `gnss_status`, optional binary stream; `.ubx` logging supported.
- IMU BNO055 (I2C) with calibration: publishes `/imu/data`, `/imu/raw`, `/imu/mag`, `/imu/temp`.
- Sonar depth acquisition (NMEA 0183 device or Imagenex 852): publishes `depth` (and `depth_enu` where applicable).
- I2C board and LED/status handling: `i2c_controller`.
- System vitals: `raspberrypi_vitals` (CPU temp/load, RAM/HDD, uptime, battery voltage).
- State aggregation: `state_controller` combines GNSS/IMU/sonar/vitals into `state`.
- Logging: `logger_text_node` with file rotation, modes (Always/Manual/Speed), geofence and optional API transfer.
- WebSockets/UI: `hydroball_config_websocket`, `hydroball_data_websocket`, `hydroball_files_websocket`.
- Diagnostics: `diagnostics` WebSocket diagnostics and communication checks.
- Shared launch args: `loggerPath`, `configPath`, `gpsdIp`, `gpsdPort`.

### Hydrobox‑Specific
- Multiple hardware and GNSS variants: RPi and Rock profiles; GNSS receivers include Piksi and Mosaic X5 in addition to ZED‑F9P (see `launch/Hydrobox/*`).
- Optional “IMU null” profiles (no IMU).
- Imagenex 852 Python driver variant in some profiles (no trigger control).
- Optional rosbag record of all topics in certain profiles.

### Hydroball‑Specific
- Power control and safe shutdown: `power_control` for rails, `power_management` triggers graceful shutdown on low voltage.
- Imagenex 852 C++ driver with trigger control: `trigger_mode` (auto|manual), `manual_ping_rate`, `data_points`.
- Alternate sonar ISA500 via NMEA device (`nmea_device_node`).
- Explicit sonar datagram extension `.img852` configured in logger params.

## Build
Prereqs (excerpt):
- ROS Noetic, catkin, g++, python3-rosdep, ros-noetic-tf2-geometry-msgs, ros-noetic-mavros, ros-noetic-sbg-driver
- Tools: `python3-wstool`, `python3-rosinstall`, `build-essential`, `ninja-build` (for Cartographer), `gpsd` and `gpsd-clients` (x64)

Steps (VM, per `README.md`):
1) Clone into `/opt`: `/opt/Poseidon`
2) Run install scripts for your target, e.g.: `install/stages/1-base-ros-noetic.sh`, `2-x64.sh`, `3-network.sh`, `4-x64.sh`, `5-finalize.sh`
3) Source ROS: `source /opt/ros/noetic/setup.bash`
4) Build workspace: `cd src/workspace && catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;...;lidar_filtering"`
5) Source workspace: `source devel/setup.bash`

Notes:
- The workspace toplevel `CMakeLists.txt` is a symlink to Catkin’s `toplevel.cmake`.
- Embedded targets may use a blacklist to reduce footprint.

## Run
- Simulator: `roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch`
- Production: choose a profile under `launch/Hydrobox/*` or `launch/Echoboat/*`, or enable the systemd `ros` service (see `install/stages/4-x64.sh`).
- Web UI: served from `www/webroot` (see VM network scripts). Open `http://localhost`.

## Application Configuration
- File: `config.txt` at repo root. Common keys:
  - `apiKey`, `apiServer`, `apiPort`, `apiUrlPath`
  - `geofence` (WKT), sensor offsets (`headingOffset`, `pitchOffset`, `rollOffset`)
  - Sonar: `sonarAbsorbtion`, `sonarPulseLength`, `sonarRange`, `sonarSerialBaudRate`, `sonarStartGain`
  - Logging: `logRotationIntervalSeconds`, `loggingMode` (1=Always ON, 2=Manual, 3=Speed based)
  - Network: `wifiSSID`, `wifiPassword`, `wifiTransferEnabled`

Launch files commonly pass `configPath` and `loggerPath` to WebSocket/logger nodes.

## System Service (systemd)
- Service `ros` created by `install/stages/4-x64.sh` with `ExecStart=/opt/Poseidon/launchROSService.sh`.
- Dependencies: `gpsd.service`, `hwrtc.service` (if present)
- Control: `sudo systemctl enable|start|stop ros`

## Tests and CI
- Unit tests: `catkin_make run_tests -j1` (see README blacklist examples).
- Jenkins: `Jenkinsfile` runs `scripts/vm_tests` and collects `build/test_results/angles/gtest-utest.xml`.

## Useful Scripts (examples)
- Rosbag record/replay: `tools/ROSBag-Eample/*.sh`, `launch/Rosbag/*`
- Lidar georeferencing: `tools/Georef_script/*.bash`, `logger/lidarGeoreferencer` (after build)
- Rosbag editing (python/C++): `tools/Edit-Rosbag/*`
- GNSS ZED‑F9P config/validate: `tools/GNSS-Config-Files/ZED-F9P/*`
- I2C/Diagnostics: `tools/i2c/*`, package `diagnostics`

## Web UI
- Static assets in `www/webroot` (SB Admin 2, MIT): `index.html`, `status.html`, `settings.html`, `map.html`, etc.
- WebSocket nodes (`hydroball_*_websocket`) expose configuration, data and files to the UI.

## Network and Ports (common)
- Internal API: `apiPort` (8080 default in `config.txt`).
- GPSD: 2947 (default), passed via launch args.
- Web: local HTTP server; Wi‑Fi file transfer via dedicated scripts/services.

## Data and Logs
- Default recording folder: `/opt/Poseidon/www/webroot/record/` (see `loggerPath` launch arg).
- GNSS binary datagrams may use `.ubx` (see `/logger/fileExtensionForGpsDatagram`).

## Nodes and Topics
Core nodes used by main launch profiles, with their primary ROS interfaces.

- i2c_controller_node (package `i2c_controller`)
  - Publishes: `led_control` (std_msgs/String)
  - Services (server): `i2c_controller_service` (i2c_controller_service/i2c_controller_service)
  - Services (client): `get_logging_status` (logger_service)
  - Args: `boardVersion` (e.g., 2.2)

- power_management_node (package `power_management`)
  - Services (client): `i2c_controller_service` → action `get_voltage`
  - Behavior: triggers graceful shutdown via systemd if voltage is critical

- raspberrypi_vitals_node (package `raspberrypi_vitals`)
  - Publishes: `vitals` (raspberrypi_vitals_msg/sysinfo)

- hydroball_config_websocket_node (package `hydroball_config_websocket`)
  - Publishes: `configuration` (setting_msg/Setting)
  - Services (server): `get_configuration` (setting_msg/ConfigurationService), `zero_imu_offsets` (setting_msg/ImuOffsetService)
  - Args: `configPath` to `config.txt`

- gpsd_client (package `gps_umd`)
  - Publishes: `fix` (sensor_msgs/NavSatFix)
  - Args: `gpsdIp`, `gpsdPort`

- gnss_zed_f9p_node (package `gnss_zed_f9p`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix)
  - Publishes: `speed` (nav_msgs/Odometry), `gnss_bin_stream` (binary_stream_msg/Stream), `gnss_status` (gnss_status_msg/GnssDiagnostic)
  - Args: `loggerPath` (output folder), `gnssPortPath` (serial device)

- gnss_mosaic_x5_node (package `gnss_mosaic_x5`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix)

- sonar_nmea_0183_tcp_client (package `sonar_nmea_0183_tcp_client`)
  - Publishes: `depth` (geometry_msgs/PointStamped), `fix` (sensor_msgs/NavSatFix), `speed` (nav_msgs/Odometry)
  - Subscribes: `configuration` (setting_msg/Setting)
  - Variants: `nmea_network_node` (TCP) and `nmea_device_node` (serial)

- sonar_imagenex852 (package `sonar_imagenex852`)
  - Publishes: `depth` (geometry_msgs/PointStamped), `depth_enu` (geometry_msgs/PointStamped)
  - Subscribes: `configuration` (setting_msg/Setting)

- sonar_imagenex852_c_node (package `sonar_imagenex852_c`)
  - Publishes: `depth` (geometry_msgs/PointStamped), `depth_enu` (geometry_msgs/PointStamped), `sonar_bin_stream` (binary_stream_msg/Stream)
  - Subscribes: `configuration` (setting_msg/Setting)

- sonar_dummy_node (package `sonar_dummy`)
  - Publishes: `depth` (geometry_msgs/PointStamped)

- imu_bno055 (executable `bno055_i2c_node`, package `imu_bno055`, namespace `imu`)
  - Publishes: `/imu/data` (sensor_msgs/Imu), `/imu/raw` (sensor_msgs/Imu), `/imu/mag` (sensor_msgs/MagneticField), `/imu/temp` (sensor_msgs/Temperature), `/imu/status` (diagnostic_msgs/DiagnosticStatus)
  - Params: `device`, `address`, `frame_id`, `calibrationFile`

- imu_dummy_node (package `imu_dummy`)
  - Publishes: `imu/data` (sensor_msgs/Imu)

- state_controller_node (package `state_controller`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix), `/imu/data` (sensor_msgs/Imu), `depth` (geometry_msgs/PointStamped), `vitals` (raspberrypi_vitals_msg/sysinfo)
  - Publishes: `state` (state_controller_msg/State)
  - Services (server): `get_state` (state_controller_msg/GetStateService)

- logger_text_node / logger_binary_node (package `logger`)
  - Subscribes: `fix`, `imu/data`, `depth`, `speed`, `configuration`, `velodyne_points`, `gnss_bin_stream`, `vitals`, `sonar_bin_stream`
  - Services (server): `get_logging_status`, `toggle_logging`, `get_logging_mode`, `set_logging_mode`, `trigger_transfer` (logger_service)
  - Params: `/logger/fileExtensionForGpsDatagram`, `/logger/fileExtensionForSonarDatagram`
  - Args: `loggerPath` (output folder)

- diagnostics_websocket_node (package `diagnostics`)
  - Serves diagnostics over WebSocket (default port 9099)
  - Internally probes topics: `fix`, `/imu/data`, `depth`, `gnss_bin_stream` and system state

- hydroball_data_websocket_node (package `hydroball_data_websocket`)
  - Subscribes: `state` (state_controller_msg/State), `gnss_status` (gnss_status_msg/GnssDiagnostic)
  - Services (client): `get_logging_status`, `toggle_logging`, `get_logging_mode`, `set_logging_mode`
  - Args: `loggerPath` (for exposing recent logs over WS)

- hydroball_files_websocket_node (package `hydroball_files_websocket`)
  - Purpose: expose files/recordings over WebSocket (no primary ROS topics)
  - Args: `loggerPath`

- wifi_file_transfer_config_node (package `wifi_file_transfer_config`)
  - Subscribes: `configuration` (setting_msg/Setting)
  - Services (client): `get_configuration` to fetch `wifiSSID`, `wifiPassword`, `wifiTransferEnabled`

### Tests (auto-generated)

Note: Coverage percentages are intentionally omitted.

- Package `diagnostics`
  - Node `diagnostics_node` (C++) — Tests: no tests detected
  - Node `diagnostics_websocket_node` (C++) — Tests: no tests detected
- Package `echoboat_odometry`
  - Node `echoboat_odometry_node` (C++) — Tests: no tests detected
- Package `gnss_dummy`
  - Node `gnss_dummy_node` (C++) — Tests: test_gnss_dummy.cpp
- Package `gnss_mosaic_x5`
  - Node `gnss_mosaic_x5_node` (C++) — Tests: gnss_mosaic_x5_test.launch, test_gnss_mosaic_x5_node.cpp
- Package `gnss_zed_f9p`
  - Node `gnss_zed_f9p_node` (C++) — Tests: gnss_zed_f9p_test.launch, test_gnss_zed_f9p.cpp
- Package `hydroball_config_websocket`
  - Node `hydroball_config_websocket_node` (C++) — Tests: test_hydroball_config_websocket.cpp
- Package `hydroball_data_websocket`
  - Node `hydroball_data_websocket_node` (C++) — Tests: hydroball_data_websocket_test.launch, test_hydroball_data_websocket.cpp
- Package `hydroball_files_websocket`
  - Node `hydroball_files_websocket_node` (C++) — Tests: test_hydroball_files_websocket.cpp
- Package `i2c_controller`
  - Node `i2c_controller_node` (C++) — Tests: no tests detected
- Package `imu_bno055`
  - Node `bno055_i2c_node` (C++) — Tests: imu_bno055_node.test, test_imu_bno055_node.cpp
  - Node `bno055_i2c_calibration` (C++) — Tests: imu_bno055_node.test, test_imu_bno055_node.cpp
- Package `imu_dummy`
  - Node `imu_dummy_node` (C++) — Tests: test_imu_dummy.cpp
- Package `imu_null`
  - Node `imu_null_node` (C++) — Tests: test_imu_null.cpp
- Package `inertial_sense_ros`
  - Node `inertial_sense_node` (C++) — Tests: no tests detected
- Package `ins_piksi`
  - Node `ins_piksi_node` (C++) — Tests: ins_piksi_test.launch, test_ins_piksi_node.cpp
- Package `lidar_filtering`
  - Node `lidar_filtering_node` (C++) — Tests: no tests detected
- Package `logger`
  - Node `logger_text_node` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
  - Node `logger_binary_node` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
  - Node `poseidonReader` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
  - Node `lidarGeoreferencer` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
  - Node `calib_printer` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
  - Node `vital_printer` (C++) — Tests: logger_binary_node_test.launch, logger_text_node_test.launch, test_logger_binary_node.cpp, test_logger_text_node.cpp
- Package `power_control`
  - Node `main.py` (Python) — Tests: no tests detected
- Package `power_management`
  - Node `power_management_node` (C++) — Tests: no tests detected
- Package `raspberrypi_vitals`
  - Node `raspberrypi_vitals_node` (C++) — Tests: test_raspberrypi_vitals.cpp
- Package `sonar_dummy`
  - Node `sonar_dummy_node` (C++) — Tests: test_sonar_dummy.cpp
- Package `sonar_imagenex852`
  - Node `main.py` (Python) — Tests: sonar_imagenex852_node.test, test_sonar_imagenex852_node.cpp
- Package `sonar_imagenex852_c`
  - Node `sonar_imagenex852_c_node` (C++) — Tests: no tests detected
- Package `sonar_nmea_0183_tcp_client`
  - Node `nmea_device_node` (C++) — Tests: no tests detected
  - Node `nmea_network_node` (C++) — Tests: no tests detected
  - Node `device_node` (C++) — Tests: no tests detected
  - Node `network_node` (C++) — Tests: no tests detected
- Package `state_controller`
  - Node `state_controller_node` (C++) — Tests: no tests detected
- Package `tf`
  - Node `tf_empty_listener` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `tf_echo` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `tf_change_notifier` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `tf_monitor` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `static_transform_publisher` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `transform_listener_unittest` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp, transform_listener_unittest.cpp, transform_listener_unittest.launch
  - Node `test_message_filter` (C++) — Tests: test_message_filter.cpp, test_message_filter.xml, tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `testListener` (C++) — Tests: testListener.cpp, tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `testBroadcaster` (C++) — Tests: testBroadcaster.cpp, tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
  - Node `tf_speed_test` (C++) — Tests: tf_benchmark.cpp, tf_unittest.cpp, tf_unittest_future.cpp
- Package `video_recorder`
  - Node `video_recorder_node` (C++) — Tests: no tests detected
- Package `water_infiltration`
  - Node `water_infiltration.py` (Python) — Tests: no tests detected
- Package `wifi_file_transfer_config`
  - Node `wifi_file_transfer_config_node` (C++) — Tests: no tests detected
  - Behavior: applies Wi‑Fi config via `nmcli` and (re)activates connection

- echoboat_odometry_node (package `echoboat_odometry`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix), `/imu/data` (sensor_msgs/Imu), `/velodyne_points` (sensor_msgs/PointCloud2), `/depth` (geometry_msgs/PointStamped)
  - Publishes: `odom` (nav_msgs/Odometry), `depth_ned` (geometry_msgs/PointStamped), TF `map → base_link`

- lidar_filtering_node (package `lidar_filtering`)
  - Subscribes: `velodyne_points` (sensor_msgs/PointCloud2)
  - Publishes: `filtered_lidar` (sensor_msgs/PointCloud2)

- video_recorder_node (package `video_recorder`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix)
  - Purpose: video capture/overlay (no main ROS outputs)

- ins_piksi_node (package `ins_piksi`)
  - Subscribes: `fix` (sensor_msgs/NavSatFix)

Common topic names used across nodes: `fix`, `/imu/data`, `depth`, `speed`, `state`, `vitals`, `velodyne_points`.

## Additional Docs
- SLAM/Mapping: `cartographer.md`, `gmapping.md`
- Hardware and manuals: `docs/` (Devices, Firmware, PCB, User Manual, etc.)

## Practices
- Always source ROS and the workspace (`source /opt/ros/noetic/setup.bash` then `source src/workspace/devel/setup.bash`).
- When adding a package, follow standard Catkin layout.
- Keep secrets out of version control; `config.txt` contains placeholders by default.
