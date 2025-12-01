# hydroball_data_websocket_node

## Overview
- ROS node name: `hydroball_websocket_controller`.
- Hosts a WebSocket server (default port `9002`) that streams telemetry from ROS to the web UI and proxies logging control commands.
- Telemetry payloads are JSON assembled from `state_controller_msg/State` and `gnss_status_msg/GnssDiagnostic`.

## Inputs
- `state` (`state_controller_msg/State`): position, imu, depth, vitals; each field is checked for valid header/seq before inclusion.
- `gnss_status` (`gnss_status_msg/GnssDiagnostic`): forwarded as a separate JSON message (`type: "gnss_status"`).
- TF: transform `base_link` → `imu` is used to compute heading/pitch/roll (via `QuaternionUtils::applyTransform`).
- Services:
  - `get_logging_status` (`logger_service/GetLoggingStatus`)
  - `toggle_logging` (`logger_service/ToggleLogging`)
  - `get_logging_mode` (`logger_service/GetLoggingMode`)
  - `set_logging_mode` (`logger_service/SetLoggingMode`, wired but not used in current commands)

## Outputs (WebSocket messages)
- Telemetry JSON (sent when `state.stamp` advances > 200 ms):
  - `position`: `[lon, lat]` or empty array if no fix.
  - `gnssFix`: NavSatStatus status code.
  - `attitude`: `[heading_deg(0-360), pitch_deg, roll_deg]` or empty if IMU missing/TF fails.
  - `depth`: `[z]` or empty.
  - `vitals`: `[cpuTemp, cpuLoad, freeRam, freeHdd, uptime, temp, voltage, humidity, ledState]` plus `status` string.
  - `recordingInfo`: `{status: bool}` and `loggingMode`: `{the_mode_is: int}` appended per broadcast.
- GNSS status JSON: `{type:"gnss_status", fix_type, diff_soln, carr_soln, num_sv, h_acc, v_acc}` pushed on every `gnss_status` message.

## Commands (WebSocket client → server)
- `{"command":"getLoggingInfo"}` → replies with current `recordingInfo` and `loggingMode`.
- `{"command":"startLogging"}` → calls `toggle_logging` (enable), then replies with updated logging info.
- `{"command":"stopLogging"}` → calls `toggle_logging` (disable), then replies.
- (Mode change not exposed yet; `set_logging_mode` client exists but unused.)

## Behavior
- Maintains active client set; broadcasts telemetry to all connections.
- If logging mode is `1` (auto) and recording is off when sending telemetry, attempts to enable logging before sending.
- `run(port)` starts the ASIO server; node spins `ros::spin()` while server runs in a separate thread; on shutdown calls `stop()` to close connections.

## Usage
```bash
rosrun hydroball_data_websocket hydroball_data_websocket_node   # listens on 9002
```
- Ensure `logger_service` is running so logging services are available.
- Avoid port conflicts; adjust source if a different port is needed.

## Notes / Caveats
- No authentication on the WebSocket server; keep it on trusted networks.
- Attitude relies on TF `base_link`→`imu`; missing TF leads to empty attitude array (warns).
- Telemetry rate is gated at ~5 Hz by timestamp delta; burstier `state` messages are dropped in between.
- Update this doc if message schema or command set changes.
