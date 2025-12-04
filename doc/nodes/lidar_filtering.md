# lidar_filtering_node

## Overview
- ROS node name: `lidar_filtering`.
- Filters incoming `velodyne_points` by horizontal angle window and Euclidean distance, publishes filtered cloud on `filtered_lidar`.

## Inputs
- `velodyne_points` (`sensor_msgs/PointCloud2`): source LiDAR point cloud.

## Outputs
- `filtered_lidar` (`sensor_msgs/PointCloud2`): points that pass both filters, header copied from input.

## Arguments (required)
Provided as argv (no ROS params):
1) `minAngle` (deg) — points with heading between minAngle and maxAngle are **removed**.
2) `maxAngle` (deg)
3) `minDistance` (m) — points closer than this are removed.
4) `maxDistance` (m) — points farther than this are removed.

## Behavior
- Converts incoming PointCloud2 to PointCloud for iteration.
- For each point:
  - Compute heading `theta = atan2(y,x)` in degrees; discard if `theta` in (minAngle, maxAngle).
  - Compute distance `sqrt(x² + y² + z²)`; discard if `< minDistance` or `> maxDistance`.
- Remaining points are re-packed into PointCloud2 and published.

## Usage
```bash
rosrun lidar_filtering lidar_filtering_node -30 30 0.5 50
```
- The above removes points in front sector (-30..30 deg) and outside 0.5–50 m.

## Notes / Caveats
- No frame transforms are applied; assumes input frame matches desired filtering frame.
- Filters are hardcoded via argv; update source to use ROS params if runtime tuning is needed.
- Update this doc if additional filters (e.g., elevation) are re-enabled.
