# logger_binary_node

## Overview
- Binary logger node (ROS node name: `logger`) that writes GNSS/IMU/sonar/lidar/vitals/speed data to binary files and raw GNSS/sonar datagrams, with rotation and optional transfer.
- Requires an output folder path as argv[1]; exits with error if missing.
- Exposes a transfer service (via `LoggerBase::advertiseTransferService`).

## Topics/Subscribers (via LoggerBase callbacks)
- GNSS (`sensor_msgs/NavSatFix`): writes PACKET_POSITION.
- IMU (`sensor_msgs/Imu`): writes PACKET_ATTITUDE (heading/pitch/roll).
- Sonar (`geometry_msgs/PointStamped`): writes PACKET_DEPTH (x,y,z).
- Lidar (`sensor_msgs/PointCloud2`): writes PACKET_LIDAR (x,y,z for each point).
- Speed (`nav_msgs/Odometry`): writes PACKET_SPEED (KMH from twist.linear.y).
- Vitals (`raspberrypi_vitals_msg/sysinfo`): writes PACKET_VITALS (dynamic fields parsed from sysinfo.msg).

## Outputs / Files
- Binary log file: `<timestamp>.log` (packets for position/attitude/depth/lidar/speed/vitals).
- Raw GNSS datagrams: `<timestamp><gps_ext>` (fileExtensionForGpsDatagram).
- Raw Sonar datagrams: `<timestamp><sonar_ext>` (fileExtensionForSonarDatagram).
- Rotation: time-based (`logRotationIntervalSeconds` from LoggerBase); uses `init()/finalize()/rotate()`.
- Finalize moves files from tmp folder to output folder; optional zip+transfer if `activatedTransfer` and `can_reach_server()`.

## Services
- Transfer service advertised via `advertiseTransferService(nh)` (see LoggerBase for details).

## Behavior
- Main: requires argv[1]=outputFolder; constructs `LoggerBinary`, advertises transfer service, `ros::spin()`.
- `init()`: opens binary/raw files with timestamp names when GNSS time is bootstrapped and logging is enabled.
- `finalize()`: closes/moves files; optionally zips/transfers.
- `rotate()`: triggers finalize/init when rotation interval elapsed (checked on GNSS callback).
- Each callback ensures file open, writes packet with timestamp > last timestamp for that stream.
- Vitals writer dynamically reads field names from `/opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg` at startup to emit name/value pairs.

## Parameters / Flags (inherited from LoggerBase)
- `loggerEnabled`, `bootstrappedGnssTime`, `enableGeofence`, `insideGeofence`, `logRotationIntervalSeconds`, `tmpLoggingFolder`, `outputFolder`, `activatedTransfer`, `fileExtensionForGpsDatagram`, `fileExtensionForSonarDatagram`, etc. (see LoggerBase).
- argv[1]: output folder (required).

## Notes
- Geofence check skips sonar writes when outside.
- Rotation is GNSS-driven (1 Hz) to limit CPU.
- Ensure `/opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg` path is valid for vitals field discovery.
