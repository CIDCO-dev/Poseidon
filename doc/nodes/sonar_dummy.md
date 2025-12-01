# sonar_dummy_node

## Overview
- Simulated sonar depth publisher for testing.
- Runs at 1 Hz and emits a sine-wave depth value (`sin(seq) * 30`) on the `depth` topic.
- On startup, queries configuration via `setting_msg/ConfigurationService` for optional sonar config values (stored locally, not currently used in the published message).

## Published topics
- `depth` (`geometry_msgs/PointStamped`): `point.z` set to simulated depth; header.seq incremented each publish; header.stamp = `ros::Time::now()`.

## Services used
- Client: `get_configuration` (`setting_msg/ConfigurationService`)
  - Request: `key` (string)
  - Response: `value` (string), parsed as `uint8_t`
  - Keys queried: `sonarStartGain`, `sonarRange`, `sonarAbsorbtion`, `sonarPulseLength`.

## Parameters / Config keys
- Retrieved via `get_configuration` service:
  - `sonarStartGain` (uint8_t), default 0x06
  - `sonarRange` (uint8_t), default 32
  - `sonarAbsorbtion` (uint8_t), default 0x14 (0.2 dB for 675 kHz)
  - `sonarPulseLength` (uint8_t), default 150

## Behavior
- Main loop (`sonar_dummy_node.cpp`):
  - Initialize ROS node `sonar`.
  - Construct `Sonar` (advertises publisher, config client, loads config).
  - Loop at 1 Hz: call `run()` → publish simulated depth; `ros::spinOnce()`; sleep.

## Notes
- If the configuration service is unavailable, config values fall back to the defaults above.
- Extend this node by replacing the sine generator with real sensor input, and by applying the retrieved config to the published data if needed.
