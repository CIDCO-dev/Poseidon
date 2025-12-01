# index.html

## Overview
- Landing dashboard for DataLogger. Shows live GNSS/IMU/sonar status, depth/attitude charts, and logging controls.
- Relies on `hydroball_data_websocket_node` (port `9002`) for telemetry and for start/stop logging commands.
- Uses SB Admin 2 layout + Chart.js + `gauge.min.js` for the heading gauge.

## Data Sources (WebSocket messages)
- Telemetry (`msg.telemetry`): longitude/latitude, `gnssFix`, `attitude` `[heading, pitch, roll]`, `depth` array, `vitals` (CPU, RAM, HDD free %, uptime, temps, voltage, humidity, LED state), `status` string.
- Recording info (`msg.recordingInfo`, `msg.loggingMode`): current logging state and mode (1=Always On, 2=Manual, 3=Speed-based).
- GNSS status (`msg.type == "gnss_status"`): `fix_type`, `num_sv`, `diff_soln`, `h_acc`, `v_acc`, shown in the GNSS widget.
- WebSocket endpoint: `ws://<host>:9002` (no auth).

## UI / Behavior
- Top bar: sidebar toggle, logo, record icon (mirrors recording state).
- Logging card: shows logging mode, start/stop button (visible only in manual mode), and current status text.
- Status banners: system status alert driven by `vitals[8]` and `status`; HDD low-space banner when `vitals[3] < 1`.
- GNSS card: lat/lon display, fix status colors; GNSS detail fields populated from `gnss_status` messages.
- IMU card: heading/pitch/roll values; heading radial gauge plus pitch/roll line chart (Chart.js).
- Sonar card: current depth and line chart (depth sign inverted for plotting).
- Loading overlay is shown until telemetry arrives; WebSocket reconnects after timeouts.

## Commands (browser → WebSocket server)
- `{"command":"getLoggingInfo"}`: fetch current logging state/mode (invoked on connect).
- `{"command":"startLogging"}`: request logging on.
- `{"command":"stopLogging"}`: request logging off.
- Logging buttons call these commands; button visibility is mode-dependent.

## Scripts
- `js/dashboardscript.js`: WebSocket connect/reconnect, telemetry parsing, widget updates, logging button logic.
- Shared includes: `js/poseidon.js`, `js/script.js`, Chart.js, `gauge.min.js`, Bootstrap/SB Admin 2 assets.

## Notes / Caveats
- No authentication on the WebSocket; keep the UI on trusted networks.
- Requires `hydroball_data_websocket_node` running with logging services available; otherwise logging buttons will not work.
- If TF/IMU/GNSS data are missing, cards show warnings and zeroed plots.
