# settings.html

## Overview
- System configuration page that lists editable key/value pairs and allows saving back to the platform configuration.
- Communicates with `hydroball_config_websocket_node` over WebSocket (`ws://<host>:9004`).
- Includes a confirmation modal before persisting changes.

## Data Source / Commands (WebSocket)
- Endpoint: `ws://<host>:9004` (no auth).
- On open sends `{"command":"getConfiguration"}`; expects a response `{configuration: [{key, value}, ...]}`.
- On save sends `{"command":"saveConfiguration","configuration":[{key,value},...]}` with values taken from the form fields.

## UI / Behavior
- Dynamically builds a form from the received `configuration` array:
  - `loggingMode` renders as a select with options 1=always, 2=manual, 3=speed-based (preselects current).
  - All other keys render as text inputs prefilled with their values.
- “Save Configuration” opens a modal warning about stability; confirming calls `saveConfig()` and sends the updated configuration over the socket.
- No client-side validation beyond the special-cased logging mode.

## Scripts
- `js/settingsScript.js`: WebSocket setup, rendering config fields, save/get commands, and modal hook.
- Shared includes: `js/script.js`, SB Admin 2 assets; no charts on this page.

## Notes / Caveats
- WebSocket is unauthenticated; keep UI on trusted networks.
- Saves push raw strings; server-side validation is assumed. Update doc if config schema changes.
