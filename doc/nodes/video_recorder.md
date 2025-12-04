# video_recorder_node

## Overview
- ROS node name: `video_recorder`.
- Captures frames from the default camera (`/dev/video0`) and writes PNGs to a temporary folder, then moves them to the requested output folder after stamping EXIF GPS tags.
- GPS tags are interpolated linearly between the two most recent `NavSatFix` samples to approximate per-frame positions.

## Inputs
- `fix` (`sensor_msgs/NavSatFix`): GNSS samples (longitude, latitude, altitude) used to tag captured images. Two consecutive samples are kept to interpolate EXIF coordinates for all frames taken between them.

## Outputs
- No ROS topics/services produced. Artifacts are PNG files saved under the provided output folder, with EXIF fields:
  - `GPSMapDatum` = `WGS-84`
  - `GPSLongitude`, `GPSLatitude`, `GPSAltitude` set from interpolated GNSS.

## Parameters / Arguments
- `argv[1]` (required): output folder path. A trailing `/` is added if missing.
- Camera: hardcoded to index `0` via OpenCV.
- Temp folder: `/home/ubuntu/temp/` (frames are written here before EXIF tagging and move).

## Behavior
- On startup, opens the camera. If it cannot be opened, logs an error.
- Capture loop (`run`):
  - Reads frames continuously.
  - If at least one GNSS sample is buffered, writes `image<N>.png` into the temp folder.
  - `spinOnce()` is called each loop to process callbacks.
- GNSS callback:
  - Maintains a two-sample queue. Once two samples exist, calls `interpolateCoord()`.
- `interpolateCoord()`:
  - Lists all files in the temp folder, sorts them, and linearly interpolates coordinates across file count.
  - Adds EXIF GPS tags to each file, then moves it to the output folder (same filename).

## Usage
```bash
rosrun video_recorder video_recorder_node /path/to/output_folder
```
- Ensure `/home/ubuntu/temp/` exists and is writable, and the output folder exists or is creatable.
- Camera access may require `v4l2loopback` or udev permissions depending on hardware.

## Notes / Caveats
- No synchronization with IMU; only GNSS is used.
- Interpolation assumes constant velocity between two `NavSatFix` samples; tag accuracy depends on GNSS sample rate.
- No rotation/logging of video streams; only still PNG frames are produced.
- Update this doc if additional image formats, ROS params, or IMU integration are added.
