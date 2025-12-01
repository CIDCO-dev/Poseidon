# map.html

## Overview
- Live map view (Leaflet) showing vessel track and depth-colored hotline.
- Consumes telemetry from `hydroball_data_websocket_node` (port `9002`) and configuration (geofence WKT) from `hydroball_config_websocket_node` (port `9004`).
- Adds bathymetry shapefile overlay (`map/ne_10m_bathymetry_L_0.zip`) and optional geofence polygon.

## Data Sources (WebSocket)
- Telemetry socket `ws://<host>:9002`: expects `{telemetry:{position:[lon,lat], depth:[z], vitals:[...], ...}}`.
  - Positions are appended to the track polyline; depth (index 0) feeds the hotline color scale.
  - Autopan keeps the map centered on the latest position unless toggled off.
- Config socket `ws://<host>:9004`: on open sends `{"command":"getConfiguration"}`; looks for `configuration` array entries and renders `geofence` (WKT) onto the map.

## UI / Behavior
- Track rendering: red polyline plus hotline heatmap; depth palette runs red→yellow→green→cyan→blue→purple→black.
- Geofence: WKT polygon drawn in green with fill, bounds-fit on load.
- Autopan toggle button (top-right Leaflet control) switches between following the current position and a static view; label updates to “Auto Pan On/Off”.
- Scale bar and simple legend (Bathymetry / Geofence).
- Shapefile overlay popup shows feature properties on click.

## Scripts
- `js/mapScript.js`: Leaflet init, shapefile load, telemetry/config WebSockets, geofence draw, track/hotline updates, autopan control.
- Leaflet plugins: `leaflet.hotline.js`, `leaflet.shpfile.js`, WKT helpers (`wicket`, `terraformer-wkt-parser`).

## Notes / Caveats
- WebSockets are unauthenticated; keep UI on trusted networks.
- Depth defaults to `0` if missing; hotline still plots but color may be misleading.
- Shapefile must exist at `www/webroot/map/ne_10m_bathymetry_L_0.zip`; missing file removes the base overlay but map still works.
