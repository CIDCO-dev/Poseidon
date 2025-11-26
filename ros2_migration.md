# ROS2 Migration Tracker

Use this document to track which nodes have been migrated to ROS2 and which remain on ROS1. Keep node names unchanged when porting.

## Migrated to ROS2
- `logger`/`logger_text_node`/`logger_binary_node` (ROS2 ports using LoggerBase; logging, geofence, speed trigger, LED/i2c control, zip rotation, HTTP(S) transfer of zip archives)
- `stateControl` (ROS2 Python port in `state_controller`, publishes `state`, serves `get_state`)
- `hydroball_config_websocket` (ROS2 WebSocket port in `hydroball_ws`)
- `hydroball_data_websocket` (ROS2 WebSocket port in `hydroball_ws`; telemetry JSON, gnss status broadcast, logging control)
- `hydroball_files_websocket` (ROS2 WebSocket port in `hydroball_ws`; file list/delete/publishfiles with transfer status)
- `diagnostics` (ROS2 WebSocket port in `hydroball_ws`; running-nodes list, diagnostics entries emitted)
- `raspberrypi_vitals` (ROS2 vitals publisher with i2c controller service, warning/critical thresholds, LED integration)
- `gnss_dummy` (ROS2 dummy GNSS publisher)
- `imu_dummy` (ROS2 dummy IMU publisher at 200 Hz)
- `imu_null` (ROS2 null IMU publisher)
- `power_control` (ROS2 GPIO power/LED control node)
- `power_management` (ROS2 voltage monitor with graceful shutdown)
- `sonar_dummy` (ROS2 dummy sonar publisher)
- `sonar_imagenex852_c` (ROS2 port: Imagenex 852 serial driver with depth/ENU and binary stream)
- `sonar_nmea_0183_tcp_client` (ROS2 NMEA-0183 TCP client publishing depth/fix/speed)
- `wifi_file_transfer_config` (ROS2 WiFi config manager using nmcli)
- ROS2 install script now installs gpsd/chrony/ros-jazzy-gpsd-client and applies shared chrony config; lighttpd serves `/opt/Poseidon/www/webroot`.
- `imu_bno055` (Upstream ROS2 driver `ros-jazzy-bno055` installed; local package kept as fallback)
- `gnss_zed_f9p` (ROS2 port: reads UBX via pyserial when available, publishes speed/gnss_status/binary stream, rotates UBX logs; falls back to dummy if serial unavailable)
- `i2c_controller` (ROS2 port with HIH8130/INA238 reads and LED control topic)

## Not Yet Migrated (ROS1)
- All other ROS1 nodes (update this list as you migrate)

## Notes
- Maintain original node names during migration for compatibility.
- Update the tables above as soon as a node is ported or retired.***
