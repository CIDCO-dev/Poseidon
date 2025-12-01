# imu_null_node

## Overview
- Publishes a zeroed IMU orientation at high rate (200 Hz) on `imu/data`.
- Useful as a null/identity IMU source for testing.
- Node name: `imu`.

## Published topics
- `imu/data` (`sensor_msgs/Imu`):
  - `orientation`: RPY = (roll=0, pitch=0, yaw=0) converted to quaternion.
  - `header.seq`: incremented each publish.
  - `header.stamp`: `ros::Time::now()`.

## Services / subscriptions
- None.

## Parameters
- None; outputs are hardcoded zeros.

## Behavior
- Constructor: advertises `imu/data`.
- Main loop (200 Hz): `run()` builds `sensor_msgs/Imu` with zero orientation and publishes it.
- `message()` helper can publish arbitrary RPY if invoked manually (not used in main loop).

## Notes
- Update this doc if the node gains parameters, covariance, or additional fields.
