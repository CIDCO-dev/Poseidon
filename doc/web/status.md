# status.html

## Overview
- System status page showing vitals (uptime, CPU, RAM, disk, humidity, battery, internal temperature).
- Receives telemetry via `hydroball_data_websocket_node` (WebSocket `ws://<host>:9002`) and renders progress bars/labels.
- Uses SB Admin 2 layout; sidebar is shared with other pages.

## Data Source (WebSocket message)
- Expects JSON with `telemetry.vitals` (array from `state_controller_msg/State` forwarded by `hydroball_data_websocket_node`):
  - `vitals[0]` CPU temperature (°C)
  - `vitals[1]` CPU load (%)
  - `vitals[2]` free RAM (%); page displays used RAM as `100 - vitals[2]`
  - `vitals[3]` free HDD (%); page displays used disk as `100 - vitals[3]`
  - `vitals[4]` uptime (seconds)
  - `vitals[5]` internal temperature (°C) — hidden if `-555`
  - `vitals[6]` battery voltage (V) — hidden if `-555`
  - `vitals[7]` humidity (%) — hidden if `-555`
  - (Index 8 LED state exists in the websocket payload but is unused here.)

## UI / Behavior
- Uptime formatted as `D HH:MM:SS`.
- CPU load bar: green when <90%, red otherwise.
- CPU temperature bar: green <75°C, yellow 75–80°C, red >80°C.
- Memory bar shows usage (`100 - freeRam%`); green when free >10%, red otherwise.
- Disk bar shows usage (`100 - freeHdd%`); green >20% free, yellow 5–20%, red ≤5%.
- Humidity, battery, temperature bars are hidden if the value is `-555` (sensor not available). Otherwise:
  - Humidity: green <40%, yellow 40–60%, red >60%.
  - Battery: voltage-based coloring/width (danger ≤11.7V or ≥15.4V, warning 11.7–11.9V or 13.5–15.4V, success 11.9–13.5V).
  - Temperature (internal): green <60°C, yellow 60–75°C, red >75°C.
- A `systemStatus` alert element exists but is not populated by `statusScript.js`.

## Scripts
- `js/statusScript.js`: opens WebSocket, parses `telemetry.vitals`, updates bars/labels, hides optional sensors on sentinel values.
- Shared includes: `js/poseidon.js`, `js/script.js`, SB Admin 2, Chart.js, `gauge.min.js` (gauge not used on this page).

## Notes
- No authentication on the WebSocket; keep the UI on trusted networks.
- If telemetry is missing or malformed, the page leaves values unchanged/blank.
