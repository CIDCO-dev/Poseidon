# goal.html

## Overview
- Goal planner page for editing a list of waypoint goals and viewing them on a Leaflet map.
- Uses a WebSocket connection (hardcoded `ws://192.168.1.16:9003` in `goal.html`) to fetch and modify goals. Intended to align with the files WebSocket (`hydroball_files_websocket_node`); adjust host/port as needed.

## Data Source / Messages
- WebSocket sends/receives JSON arrays named `goal_planner`:
  - Incoming: `{goal_planner: [[lat, lon], ...]}` populates the table and draws the polyline on the map.
  - Outgoing commands (JSON strings built in JS):
    - `{"go_add":[lat, lon]}`: append a goal.
    - `{"go_ins":[index, lat, lon]}`: insert at a selected row (first checked box).
    - `{"go_del":[index]}`: delete selected goals (sent per checked row, iterating from end).
  - Selection toggling is client-side only; server must respond with an updated `goal_planner` to refresh the UI.

## UI / Behavior
- Map: Leaflet map with shapefile overlay (`/map/TM_WORLD_BORDERS-0.3.zip`), draws a red polyline through goals; pans to last goal on update.
- Table: lists latitude/longitude per goal with checkboxes.
- Buttons:
  - Select all/none (inverts current selection).
  - Delete (opens confirmation modal, then sends `go_del` for checked rows).
  - Add @ end (modal collects lat/lon, sends `go_add`).
  - Add @ line (modal collects lat/lon, inserts at first checked row via `go_ins`).
  - Edit button exists but is disabled (no implementation).
- No client-side validation beyond simple `isNaN` checks; incorrect values are silently ignored.

## Scripts
- Inline JS in `goal.html`: WebSocket connect, map setup, shapefile load, goal list rendering, add/insert/delete helpers.
- Leaflet plugins: `leaflet.shpfile.js` for border overlay; no external mapScript used here.

## Notes / Caveats
- WebSocket URL is hardcoded to `ws://192.168.1.16:9003`; update to `ws://<host>:9003` or the appropriate service before deployment.
- No authentication/TLS. Server must echo updated `goal_planner` for UI to refresh after commands.
- Edit flow is unimplemented; add validation and server-side checks as needed for production use.
