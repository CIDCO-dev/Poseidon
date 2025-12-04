# raspberrypi_vitals_node

## Overview
- Publishes Raspberry Pi vitals at 1 Hz on `vitals` (CPU temp/load, RAM/HDD usage, uptime, humidity/temperature/voltage via I2C service, LED state, status).
- Node name: `hbv`.
- Uses local file/proc reads for system metrics, and an I2C controller service for environmental/battery readings and LED control.

## Published topics
- `vitals` (`raspberrypi_vitals_msg/sysinfo`):
  - `cputemp`: `/sys/class/thermal/thermal_zone0/temp` (°C)
  - `cpuload`: `/proc/loadavg` normalized by 8 cores (%)
  - `freeram`: `/proc/meminfo` free/total (%)
  - `freehdd`: `df --output=pcent /` inverted (% free)
  - `uptime`: `/proc/uptime` first value (seconds)
  - `humidity`, `temperature`, `voltage`, `ledstate`: from I2C service calls (see below)
  - `status`: text set by warning/critical checks (“Normal” or warning/critical messages)

## Services used
- Client: `i2c_controller_service` (`i2c_controller_service/i2c_controller_service`)
  - Actions requested:
    - `get_humidity` → `humidity`
    - `get_temperature` → `temperature`
    - `get_voltage` → `voltage`
    - `get_led_state` → `ledstate`
    - `led_error` (triggered on critical)
    - `led_warning` (triggered on warning)
  - On service failure: `humidity/temperature/voltage` set to `-555.0` sentinel.

## Status thresholds
- Critical (sets status, calls `led_error`):
  - `freehdd < 5%` (and >0)
  - `voltage <= 11.7V` (and >0)
  - `voltage >= 15.4V`
  - `cputemp >= 80°C`
- Warning (sets status, calls `led_warning`):
  - `voltage <= 15.4V` and >13.5V (charging)
  - `voltage <= 11.9V` and >0 (low)
  - `freehdd < 20%` and >0 (low space)
  - `cputemp >= 75°C` (high temp)
- Otherwise: status “Normal”.

## Parameters / configuration
- None via ROS params. Debug flags are hardcoded (`debug_mode`, `debug_mode_warning_critical`).

## Behavior
- Constructor: advertises `vitals`, creates I2C service client, waits for service.
- Main loop (1 Hz): read system metrics, call I2C actions, evaluate status/LED, publish `sysinfo`.

## Notes
- Update this doc if adding parameters, changing thresholds, or altering published fields/topics.
