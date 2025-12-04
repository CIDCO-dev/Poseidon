# diagnostics_websocket_node

## Overview
- ROS node name: `diagnostics`.
- Hosts a WebSocket server (port `9099`) that serves two commands to the web UI:
  - `updateDiagnostic`: runs all diagnostics tests and returns the results.
  - `getRunningNodes`: returns the list of currently running ROS nodes.
- Uses a set of diagnostic tests (GNSS comm/fix, IMU comm/calibration, sonar comm, clock, API connection, serial number pattern, GNSS binary stream).

## Inputs
- Relies on diagnostics test classes to query ROS state (topics/services); no ROS topics are directly subscribed in this node.

## Outputs (WebSocket messages)
- `diagnostics` array: produced on `updateDiagnostic` command. Each test calls `do_test()` then serializes itself into JSON via `to_json`.
- `running_nodes` array: produced on `getRunningNodes`, listing node names from `ros::master::getNodes()`.

## Behavior
- WebSocket server with no TLS, reuse-addr enabled, logging disabled.
- On each command, builds a JSON document and broadcasts to all connected clients.
- Stores connections in a set protected by a mutex; `send_json` broadcasts to all.
- `stop()` cleanly stops listening and closes connections when the node shuts down.

## Parameters
- Port is hardcoded to `9099` in `diagnostics_websocket_node.cpp`. Change the source to use a ROS param if needed.

## Usage
```bash
rosrun diagnostics diagnostics_websocket_node   # listens on 9099
```
- Open the web UI diagnostics page to issue `updateDiagnostic` / `getRunningNodes` commands.

## Notes / Caveats
- No authentication or TLS; keep on trusted networks.
- If new diagnostic tests are added/removed, update the list in `DiagnosticsServer` construction and this doc.
- Running node list depends on ROS master visibility; if master is remote or filtered, results may differ.
