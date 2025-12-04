# bno055_i2c_node / bno055_i2c_calibration

## Overview
- ROS nodes: `bno055_i2c_node` (runtime) and `bno055_i2c_calibration` (one-shot calibration). Launch files place them in the `imu` namespace.
- Driver reads a Bosch BNO055 over I2C, applies saved calibration offsets, publishes fused and raw IMU data plus diagnostics, and exposes reset/calibrate services.
- Calibration node monitors calibration status until complete, then writes offsets to a file for reuse by the runtime node.

## Topics (published)
- `data` (`sensor_msgs/Imu`): fused orientation (unit quaternion), fused linear acceleration (m/s^2), angular velocity (rad/s) derived from fused data.
- `raw` (`sensor_msgs/Imu`): raw linear acceleration (m/s^2) and angular velocity (rad/s) direct from sensor registers.
- `mag` (`sensor_msgs/MagneticField`): magnetic field (scaled by 1/16 from raw counts).
- `temp` (`sensor_msgs/Temperature`): onboard IMU temperature in °C.
- `status` (`diagnostic_msgs/DiagnosticStatus`): calibration/self-test/interrupt/system status/errors (published every 50 spins).

## Services (provided)
- `calibrate` (`std_srvs/Trigger`): placeholder (returns true; main calibration flow is via `bno055_i2c_calibration`).
- `reset` (`std_srvs/Trigger`): soft reset + reapply calibration + mode setup.

## Parameters
- `~device` (string, default `/dev/i2c-1`): I2C device path.
- `~address` (int, default `0x28`): I2C address (`BNO055_ADDRESS_A`).
- `~frame_id` (string, default `imu`): frame set on all published messages.
- `~calibrationFile` (string, default `/opt/Poseidon/calibration.dat`): binary offsets file; loaded on startup and written by calibration node when fully calibrated.
- `~rate` (int, default `100`): main loop frequency for `bno055_i2c_node` (watchdog refreshed on successful spins).

## Behavior
- On start: opens I2C device, verifies chip ID, loads calibration offsets if the file exists, remaps axes to ENU, sets operation mode to `NDOF`, and begins publishing.
- Each spin reads two I2C blocks (0x20 + 0x13 bytes), publishes fused/raw IMU, mag, and temperature messages; publishes diagnostics periodically.
- Watchdog is refreshed only when a spin succeeds; I2C read failures will stop refresh and let the watchdog terminate the node.
- `bno055_i2c_calibration`: opens the sensor, enters a loop printing calibration progress; when all sensors are calibrated (`0xFF`), dumps the calibration block to `calibrationFile` and exits.

## Usage
```bash
# Runtime
roslaunch imu_bno055 imu.launch          # starts bno055_i2c_node in ns=imu

# Calibration (collect offsets, writes calibrationFile)
roslaunch imu_bno055 calibrate.launch    # runs bno055_i2c_calibration until fully calibrated
```
- Ensure the I2C device and address match the hardware wiring.
- Persist the generated calibration file on the target so runtime can load it.
