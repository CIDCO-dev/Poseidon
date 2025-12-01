# hydroball_config_websocket_node

## Overview
- Websocket-based configuration server for Hydroball. Bridges config between ROS and web UI on port 9004.
- Loads configuration from a file (path passed as argv[1]) at startup; persists updates back to that file.
- Broadcasts IMU static transform from stored offsets and publishes config key/value updates.
- Provides services to read configuration and zero IMU offsets.
- Node name: `hydroball_config_websocket`.

## Published topics
- `configuration` (`setting_msg/Setting`): publishes key/value pairs when broadcasting config changes.

## Services offered
- `get_configuration` (`setting_msg/ConfigurationService`)
  - Request: `key` (string)
  - Response: `value` (string) if key exists.
- `zero_imu_offsets` (`setting_msg/ImuOffsetService`)
  - Reads current IMU orientation from `state_controller/get_state`; stores negated yaw/pitch/roll as offsets; writes config file; rebroadcasts config and IMU transform.

## Services used
- Client: `get_state` (`state_controller_msg/GetStateService`) to fetch current IMU orientation for zeroing.

## WebSocket API (port 9004)
- Commands (JSON):
  - `{"command":"getConfiguration"}` → server sends current configuration as array of `{key,value}`.
  - `{"command":"saveConfiguration","configuration":[{"key":"...","value":"..."}]}` → updates stored configuration, writes file, broadcasts config and IMU transform, updates hotspot SSID if changed.
  - `{"command":"zeroImu"}` → invokes zero_imu_offsets service logic.
- Connections are tracked; on stop, the server closes all connections with a normal close frame.

## File I/O
- Reads config from `configFilePath` (argv[1]) on startup. Each line: `<key> <value>` (or `<key> <value...>` if spaces present).
- Writes config back on save/zero operations.

## IMU transform broadcast
- If `headingOffset`, `pitchOffset`, and `rollOffset` exist and parse as doubles, broadcasts a static transform `base_link` → `imu` using those offsets (RPY in degrees).

## Network/Hotspot update
- If `hotspotSSID` changes, updates NetworkManager connection `Hotspot` via `nmcli` (down/up) and verifies SSID.

## Parameters / arguments
- argv[1]: path to configuration file (required). Node exits with error if missing.
- Port: hardcoded to 9004 in node main when starting the websocket server.
- No ROS params used.

## Behavior
- Main node (`hydroball_config_websocket_node.cpp`): expects config path arg; starts ConfigurationServer; runs websocket server on port 9004 in a separate thread; spins at 10 Hz until shutdown; then stops server and joins thread.

## Notes
- Update this doc if commands, config schema, or services change. Ensure config file path is provided at launch. Credentials/SSIDs are handled via the config file; avoid committing real secrets.
