# ins_piksi_node

## Overview
- ROS node name: `piksi`.
- Reads Swift/Piksi SBP datagrams from a serial port and writes them to timestamped `.sbp` files.
- Waits for a valid GNSS fix on `fix` before logging to ensure GNSS time is initialized.

## Inputs
- `fix` (`sensor_msgs/NavSatFix`): gate startup; when `status.status >= 0`, logging begins.
- Serial port (argument): raw SBP stream from the Piksi receiver.

## Outputs
- No ROS topics or services are published.
- File output: `<logPath>/<YYYY.MM.DD_HHMMSS>.sbp` containing SBP datagrams (header + payload + CRC) exactly as read from the serial port.

## Arguments
- `argv[1]` (required): `logPath` directory where `.sbp` files are written.
- `argv[2]` (required): `serialDevice` path (e.g., `/dev/ttyACM0`).

## Behavior
- Subscribes to `fix` and blocks until a fix is reported.
- Opens the serial port at 460800 baud, 8N1, no flow control.
- Aligns on SBP datagram preamble (`0x55`) for ~2 seconds to clear/align the buffer.
- Main loop: read one datagram (header + payload + CRC), call `processDatagram` hook (no default publishing), and append to the current `.sbp` file.
- On file or serial open errors, logs and exits.

## Parameters
- None; baud rate, port path, and log path are provided via arguments.

## Usage
```bash
rosrun ins_piksi ins_piksi_node /opt/Poseidon/www/webroot/record/ /dev/ttyACM0
```
- Ensure the output directory exists and is writable.
- Ensure `fix` is available so startup can proceed.

## Notes
- `processDatagram` is available for extensions/tests but is a no-op in normal operation.
- Make sure the serial device has the correct permissions on the target platform.
