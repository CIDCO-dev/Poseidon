# logger_service

## Overview
- Defines the logging control services used by `logger_text_node` and `logger_binary_node` (exposed via `LoggerBase`).
- Service names are advertised in the node namespace (e.g., `/get_logging_status`, `/toggle_logging`, etc.).
- Logging modes: `1` Always On, `2` Manual, `3` Speed-Based (uses average speed vs. `speedThresholdKmh` config, default 5 km/h).

## Service Servers (provided by logger nodes)
- `get_logging_status` (`logger_service/GetLoggingStatus`): request empty; response `status` (bool) is true only when GNSS time is bootstrapped and logging is currently enabled.
- `toggle_logging` (`logger_service/ToggleLogging`): request `loggingEnabled` (bool). Response `loggingStatus` reflects current state. If GNSS time is not bootstrapped the call fails unless `/logger/allow_toggle_without_gps` is set true.
- `get_logging_mode` (`logger_service/GetLoggingMode`): request empty; response `loggingMode` (int32, values above). Used by speed-based logic to decide auto toggles.
- `set_logging_mode` (`logger_service/SetLoggingMode`): request `loggingMode` (int32). Response empty; updates in-memory mode (Always On, Manual, or Speed-Based).
- `trigger_transfer` (`logger_service/TriggerTransfer`): request empty; response `success` (bool) and `message`. Advertised via `LoggerBase::advertiseTransferService`; triggers packaging and upload of rotated logs if transfer config is set.

## Behavior Notes
- Always On: logging starts once GNSS time is available and stays on.
- Manual: logging changes only through `toggle_logging`.
- Speed-Based: average speed over the last ~120 samples is compared to `speedThresholdKmh`; crossing the threshold auto-enables/disables logging.
- Service clients include the web sockets (`hydroball_data_websocket`, `hydroball_files_websocket`), vitals/i2c controllers, and the logger tests.

## Usage
```bash
rosservice call /get_logging_status
rosservice call /toggle_logging "loggingEnabled: true"
rosservice call /set_logging_mode "loggingMode: 2"
rosservice call /trigger_transfer
```
