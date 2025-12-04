# diagnostics.html

## Overview
- Diagnostics page showing current ROS diagnostics and the list of running nodes.
- Connects to the diagnostics WebSocket (`ws://<host>:9099`) to request data and render tables.
- Uses SB Admin 2 layout; sidebar shared with other pages.

## Data Source (WebSocket messages)
- WebSocket server: `ws://<host>:9099` (no auth).
- Messages handled:
  - `{diagnostics: [ {status: bool, name: string, message: string}, ... ]}`
  - `{running_nodes: [ "node_name1", "node_name2", ... ]}`

## UI / Behavior
- “Refresh diagnostics” button triggers both:
  - `{"command":"getRunningNodes"}`
  - `{"command":"updateDiagnostic"}`
- On open, the socket automatically sends both commands.
- Diagnostics table columns: Status (✅/❌), Diagnostic name, Info (message with line breaks preserved).
- Running nodes table lists each node name in a single-column table.
- Loading spinner shows while waiting for responses.

## Scripts
- `js/diagnosticsScript.js`: WebSocket setup, send commands, parse `diagnostics`/`running_nodes` responses, build tables.
- Shared includes: SB Admin 2 assets; page does not use Chart.js or gauges.

## Notes / Caveats
- No authentication on the WebSocket; keep on trusted networks.
- If messages are missing fields, tables may be empty; the script clears previous rows before rendering.
