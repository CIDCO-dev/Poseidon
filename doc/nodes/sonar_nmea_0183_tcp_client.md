# sonar_nmea_0183_tcp_client (network_node, device_node)

## Overview
- Provides two entry points:
  - `nmea_network` (TCP client): connects to an NMEA-0183 TCP stream (default 127.0.0.1:5000).
  - `nmea_device` (serial client): reads NMEA-0183 from a serial device (default `/dev/sonar`), configurable baud rate.
- Parses depth (DBT/DPT/ADS), GNSS position (GGA), and speed/course (VTG), publishing ROS topics for downstream consumers.

## Published topics
- `depth` (`geometry_msgs/PointStamped`): depth in meters from DBT/DPT/ADS.
- `fix` (`sensor_msgs/NavSatFix`): position from GGA. Status set to -1/0/2 for no fix/GPS/DGPS.
- `speed` (`nav_msgs/Odometry`): `twist.twist.linear.y` set to ground speed km/h from VTG.

## Subscribed topics
- `configuration` (`setting_msg/Setting`) — device mode only: watches for `sonarSerialBaudRate` updates and reapplies the baud rate.

## Services used
- `get_configuration` (`setting_msg/ConfigurationService`) — device mode only: initial baud rate lookup (`sonarSerialBaudRate`).

## Parameters
### nmea_network
- `ip_address` (string, default `127.0.0.1`)
- `port` (int, default `5000`)

### nmea_device (under `/Sonar/` namespace)
- `/Sonar/device` (string, default `/dev/sonar`)
- `/Sonar/useDepth` (bool, default true) — currently depth always published when parsed.
- `/Sonar/usePosition` (bool, default false) — currently position always published when parsed.
- `/Sonar/useAttitude` (bool, default false) — attitude not parsed (reserved).
- `/Sonar/useAttitude` is reserved; no attitude sentences handled today.

### Serial baud rate (device mode)
- `sonarSerialBaudRate` (from configuration service) defaults to 9600 on missing/invalid value. Applied with `termios`.

## Behavior
- Checksum validation:
  - Each NMEA line is checksum-validated before parsing; invalid lines log `checksum error` and are skipped.
- Parsing:
  - GGA → publishes `fix` with converted lat/lon (deg/min to decimal), altitude, and status.
  - DBT/DPT/ADS → publishes `depth` (meters).
  - VTG → publishes `speed` with km/h.
- TCP mode:
  - Connects, sets a 5s recv timeout, reads char-by-char, and reconnects on failure with 1 Hz retry.
- Serial mode:
  - Opens device, sets baud rate from config service, listens for config updates to adjust baud, and reads char-by-char.
- State:
  - Sequence numbers increment per message type; timestamps set to `ros::Time::now()` on publish.

## Usage
```bash
# TCP client
rosrun sonar_nmea_0183_tcp_client network_node _ip_address:=192.168.0.10 _port:=5000

# Serial client
rosrun sonar_nmea_0183_tcp_client device_node _Sonar/device:=/dev/ttyUSB0
```

## Notes / Caveats
- No authentication; keep the TCP source on a trusted network.
- Char-by-char reading is inefficient but simple; may drop data if stream is very high rate.
- Attitude sentences are not parsed; `useAttitude` is a placeholder.
- Update this doc if additional NMEA sentences or params are added.
