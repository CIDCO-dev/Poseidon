# gnss_dummy_node

## Overview
- Dummy GNSS publisher (ROS node name: `gnss`) for testing; publishes simulated `NavSatFix` at 1 Hz on `fix`.
- Coordinates follow a synthetic path with noise; status alternates to simulate fix quality.

## Published topics
- `fix` (`sensor_msgs/NavSatFix`):
  - `header.seq`: incremented per publish
  - `header.stamp`: `ros::Time::now()`, nsec forced to 0
  - `status.status` and `status.service`: set to the `status` argument passed to `talk()` (alternates)
  - `longitude`, `latitude`: synthetic path (sinusoidal + noise)
  - `altitude`: `sin(seq*42+100)*10` (ellipsoidalHeight)

## Behavior
- Main loop (1 Hz):
  - For 60 iterations: `talk(-1)` (e.g., no/invalid fix)
  - For next 30 iterations: `talk(1)` (e.g., good fix)
  - Then resets counter to 30 and repeats (50/50 bias toward good fix after first loop)
- `talk(int status)`:
  - Increments `sequenceNumber`
  - Perturbs longitude/latitude with noise and sinusoids; also sets a straight-path variant (latitude from distance/earth radius) at the end
  - Calls `message(seq, lon, lat, status)`
- `message(...)`: fills `NavSatFix` and publishes.

## Services / Subscriptions
- None.

## Parameters
- None; behavior hardcoded.

## Notes
- Update this doc if path/noise or status patterns change. Ensure `fix` topic consumers can handle alternating status values and synthetic coordinates.
