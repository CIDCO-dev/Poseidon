# calibration.html

## Overview
- Calibration utility page (currently limited to IMU zeroing).
- Uses the config WebSocket (`hydroball_config_websocket_node`, `ws://<host>:9004`) to send commands.

## Actions / Commands
- “Zero IMU offsets” button calls `sendZeroImu()` → sends `{"command":"zeroImu"}` over the WebSocket.
- No other UI fields; no feedback is displayed by default (socket `onmessage` is a no-op).

## Data Source
- WebSocket: `ws://<host>:9004` (shared with settings/config). No authentication.
- Incoming messages are parsed but ignored (`processState` is empty); extend if server emits acknowledgements.

## Scripts
- `js/calibrationScript.js`: opens WebSocket, defines `sendZeroImu`, placeholder `processState`.
- Shared includes: `js/script.js`, `js/poseidon.js`, SB Admin 2 assets.

## Notes / Caveats
- No confirmation or status is shown; users must verify IMU behavior separately.
- Add server-side support and UI feedback if more calibration flows are introduced.
