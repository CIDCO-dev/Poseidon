# gnss_mosaic_x5_node

## Overview
- ROS node name: `mosaic-X5`.
- Reads Septentrio Mosaic-X5 SBF datagrams from a serial port and logs them to `.sbf` files.
- Waits for a valid GNSS fix on `fix` before starting, to ensure GNSS time is available.

## Inputs
- `fix` (`sensor_msgs/NavSatFix`): used only to gate startup; when `status.status >= 0`, logging begins.
- Serial port (argument): raw SBF stream from the receiver.

## Outputs
- No ROS topics published.
- File output: `<logPath>/<YYYY.MM.DD_HHMMSS>.sbf` containing the raw SBF datagrams (header + payload).

## Arguments
- `argv[1]` (required): `logPath` directory to write `.sbf` files.
- `argv[2]` (required): `serialDevice` path (e.g., `/dev/ttyACM0`).

## Behavior
- Subscribes to `fix` and blocks until a fix is reported.
- Opens the serial port at 115200 8N1, no flow control.
- Aligns on SBF datagram sync (`0x24 0x40`) for ~2 seconds to flush/align the stream.
- Main loop: read one SBF block (header + payload), call `processDatagram` hook (currently empty), then append to the `.sbf` file.
- On errors opening the port or file, logs and exits.

## Parameters
- None; baud rate (115200), port name, and log path are fixed via arguments.

## Usage
```bash
rosrun gnss_mosaic_x5 gnss_mosaic_x5_node /opt/logs /dev/ttyACM0
```
- Ensure the serial device is accessible (udev permissions).
- Ensure `fix` is being published so the node can start.

## Notes / Caveats
- No ROS publications or diagnostics yet; only raw SBF logging.
- `processDatagram` is a hook for future decoding; update this doc if decoding/publishing is added.
- Make sure the output directory exists and has free space.
