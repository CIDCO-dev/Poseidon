# logger_text_node

## Overview
- Text logger node (ROS node name: `logger`) that writes GNSS/IMU/sonar/lidar/vitals/speed data to per-stream text files plus raw GNSS/sonar datagrams, with rotation and optional transfer.
- Requires an output folder path as argv[1]; uses `;` as the default separator.
- Exposes a transfer service (via `LoggerBase::advertiseTransferService`).

## Topics/Subscribers (via LoggerBase callbacks)
- GNSS (`sensor_msgs/NavSatFix`): `_gnss.txt` (Timestamp, Longitude, Latitude, EllipsoidalHeight, Status, Service).
- IMU (`sensor_msgs/Imu`): `_imu.txt` (Timestamp, Heading, Pitch, Roll).
- Sonar (`geometry_msgs/PointStamped`): `_sonar.txt` (Timestamp, Depth); geofence check skips if outside.
- Lidar (`sensor_msgs/PointCloud2`): `_lidar.txt` (Timestamp, followed by x y z triples).
- Speed (`nav_msgs/Odometry`): `_speed.txt` (Timestamp, Speed from twist.linear.y).
- Vitals (`raspberrypi_vitals_msg/sysinfo`): `_vitals.txt` (Timestamp, CPU Temp, CPU Used, FREE RAM, FREE HDD, Uptime, Humidity, System Temp, Supply Voltage) every >60s; headers derived from `sysinfo.msg`.
- Raw GNSS/sonar datagrams: binary files with configured extensions.

## Outputs / Files
- Text files per stream in temp folder, then moved to output folder on finalize/rotate:
  - `<date>_gnss.txt`, `<date>_imu.txt`, `<date>_sonar.txt`, `<date>_lidar.txt`, `<date>_speed.txt`, `<date>_vitals.txt`
- Raw datagrams: `<date><gps_ext>`, `<date><sonar_ext>`
- Rotation: time-based (`logRotationIntervalSeconds` from LoggerBase); uses `init()/finalize()/rotate()`.
- Finalize closes/moves files; optional zip+transfer if `activatedTransfer` and `can_reach_server()`.

## Services
- Transfer service advertised via `advertiseTransferService(nh)` (see LoggerBase).

## Parameters / Args
- argv[1]: output folder (required).
- Separator: constructor default `";"` (hardcoded in node main).
- Other flags/paths inherited from LoggerBase (e.g., `loggerEnabled`, `bootstrappedGnssTime`, `enableGeofence`, rotation interval, temp/output folders, transfer flags, datagram extensions).

## Behavior
- Main: requires output folder; constructs `LoggerText` with separator `;`, advertises transfer service, `ros::spin()`.
- `init()`: opens all text files + raw GNSS/sonar files with timestamped names when GNSS time is bootstrapped and logging enabled.
- `finalize()`: closes/moves files; optionally zips/transfers.
- `rotate()`: time-based rotation checked on GNSS callback.
- Callbacks write formatted lines with timestamps; enforce monotonic timestamps per stream to avoid duplicates.

## Notes
- Geofence check skips sonar writes when outside.
- Vitals header is read from `/opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg`; ensure path is valid.
- Update if file formats, fields, or rotation logic change.
