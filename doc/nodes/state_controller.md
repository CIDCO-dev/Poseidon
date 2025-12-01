# state_controller_node

## Overview
- ROS node name: `stateControl`.
- Aggregates navigation, attitude, depth, and vitals into a single `state_controller_msg/State` message and republishes it on `state`.
- Exposes a service to fetch the latest state snapshot.

## Inputs
- `fix` (`sensor_msgs/NavSatFix`): GNSS position. If `status.service > 0`, stored in `state.position` and timestamp updated.
- `/imu/data` (`sensor_msgs/Imu`): attitude; copied to `state.imu` and timestamp updated.
- `depth` (`geometry_msgs/PointStamped`): depth; copied to `state.depth` and timestamp updated.
- `vitals` (`raspberrypi_vitals_msg/sysinfo`): system vitals; copied to `state.vitals` and timestamp updated.

## Outputs
- `state` (`state_controller_msg/State`): republished on every input update. Fields include the most recent copies of position/imu/depth/vitals; `stamp` is set to the last message time received.

## Services
- `get_state` (`state_controller_msg/GetStateService`): returns the current `State`.

## Behavior
- On any incoming message, stores the data under a mutex, updates `stamp`, and publishes `state`.
- Initializes `position.status.status` to `-1` so consumers know there is no fix yet.
- No throttling or filtering; a future Kalman or rate limiter could be added (TODO in code).

## Parameters
- None; topic names and queue sizes are hardcoded in the constructor.

## Usage
```bash
rosrun state_controller state_controller_node
# listen on 'state' or call:
rosservice call /get_state
```

## Notes
- Assumes publishers for `fix`, `/imu/data`, `depth`, and `vitals` are already running.
- Extend this doc if rate limiting, filtering, or additional fields are added.
