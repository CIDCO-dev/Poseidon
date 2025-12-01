# gnss_zed_f9p_node

## Overview
- ROS node name: `zedf9p`. Interfaces with a ZED-F9P GNSS receiver over serial, decodes UBX frames, publishes speed and diagnostics, and streams raw UBX bytes.
- Requires two args: `logPath` and `serialDevice` (e.g., `/dev/ttyACM0`). Exits if missing/empty.
- Runs serial read loop; waits for a valid GNSS fix (status >= 0 on `/fix`) before processing frames.

## Subscriptions
- `fix` (`sensor_msgs/NavSatFix`): used only to detect that GNSS time is bootstrapped (status >= 0 sets `bootstrappedGnssTime`).

## Publications
- `speed` (`nav_msgs/Odometry`): `twist.twist.linear.y` set to ground speed (km/h) from UBX NAV-PVT `groundSpeed` (mm/s).
- `gnss_bin_stream` (`binary_stream_msg/Stream`): raw UBX frames (sync, header, payload, checksum) with `timeStamp` (ns).
- `gnss_status` (`gnss_status_msg/GnssDiagnostic`):
  - `fix_type` ← PVT fixType
  - `diff_soln` ← PVT flags bit 2
  - `carr_soln` ← PVT flags bits 6–7
  - `num_sv` ← PVT nbSatellites
  - `horizontal_accuracy` ← PVT horizontalAccuracy (mm → m)
  - `vertical_accuracy` ← PVT verticalAccuracy (mm → m)

## Behavior
- Serial config: `/dev/...` opened with termios at 460800 baud, 8N1, no flow control.
- Parse loop:
  - Read UBX sync (0xB5 0x62), header, payload, checksum.
  - Validate checksum over header+payload.
  - For NAV-PVT (class 0x01, id 0x07): publish speed and diagnostics; also forward raw frame on `gnss_bin_stream`.
  - Other frames currently ignored (logged to `gnss_bin_stream` only if NAV-PVT).
- Rotation/logging to files is not used here (leftover members for logPath, tmpFolder, rotation interval).

## Parameters / Args
- argv[1]: `logPath` (required; currently only used in INFO logging)
- argv[2]: `serialDevice` (required; e.g., `/dev/ttyACM0`)
- Rotation interval is defined but unused; no ROS params exposed.

## Notes
- Ensure serial device is accessible (permissions). If checksum errors occur, they’re logged.
- Update this doc if additional UBX messages are decoded or if file logging is enabled.
