# hydroball_files_websocket_node

## Overview
- ROS node name: `hydroball_files_websocket`.
- Hosts a WebSocket server (port `9003`) that lists log files, deletes files, and triggers log transfer via `logger_service/TriggerTransfer`.
- Uses `logPath` argument to find files (expected to contain recording artifacts).

## Inputs
- None via ROS topics.
- Service client: `trigger_transfer` (`logger_service/TriggerTransfer`) to initiate upload/transfer.

## Outputs (WebSocket responses)
- `fileslist`: array of `[filename, record/filename]` pairs (built from `logPath` directory) when `f-list` is requested.
- `publishstatus`: status messages for transfer (with optional `done: true` on terminal state).

## Commands (WebSocket client → server)
- `{"f-list": true}`: send current file list.
- `{"delete": "<filename>"}`: delete the given file in `logPath`.
- `{"publishfiles": true}`: run a transfer workflow:
  1) Check for files present; error if none.
  2) Check internet connectivity (ping 8.8.8.8).
  3) Check API server connectivity (reads `apiServer` from `/opt/Poseidon/config.txt`, probes HTTPS).
  4) Call `trigger_transfer`; report success/failure.
  - Sends stepwise `publishstatus` messages to the caller.

## Arguments
- `argv[1]` (required): `logPath` directory containing files to manage/transfer.
- Port is hardcoded to `9003`; change source to parameterize if needed.

## Behavior
- Maintains active client set and broadcasts responses to all.
- File list and deletes operate directly on `logPath`.
- Transfer flow runs in a detached thread per request; sends incremental status updates over the same connection.
- `stop()` stops listening and closes connections when ROS shuts down.

## Usage
```bash
rosrun hydroball_files_websocket hydroball_files_websocket_node /opt/Poseidon/www/webroot/record
```
- Ensure `/opt/Poseidon/config.txt` has a non-localhost `apiServer` entry if transfers require remote API.
- Ensure `logger_service` is running to provide `trigger_transfer`.

## Notes / Caveats
- No authentication/TLS on WebSocket; keep on trusted networks.
- Delete is permanent; there is no recycle bin.
- Transfer pre-checks are simplistic (ping + HTTPS head); adapt as needed.
