# imu_dummy_node

## Overview
- ROS node name: `imu`.
- Publishes synthetic IMU orientation at 200 Hz on `imu/data` for testing/bring-up.
- Orientation is a smooth, time-varying quaternion derived from sin/cos of the sequence number.

## Outputs
- `imu/data` (`sensor_msgs/Imu`): orientation only.
  - Header: `seq` increments, `stamp` = now.
  - Orientation:
    - yaw = `sin(seq / π) * 30°`
    - pitch = `cos(seq / π) * 20°`
    - roll = `sin(seq / π) * 10°`
  - Angular velocity and linear acceleration are left at default zero.

## Inputs
- None.

## Parameters
- None; topic name and rate (200 Hz) are fixed in code.

## Behavior
- Main loop: compute orientation, publish IMU message, `ros::spinOnce()`, sleep at 200 Hz.
- Helper `message(seq, yaw, pitch, roll)` exists to publish arbitrary Euler angles if used by tests.

## Usage
```bash
rosrun imu_dummy imu_dummy_node
rostopic echo /imu/data
```

## Notes
- Intended for simulation/headless testing only; replace with a real IMU driver in production.
- Update this doc if noise models, params, or topic names change.
