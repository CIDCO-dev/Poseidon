# ROS2 Migration Tracker

Use this document to track which nodes have been migrated to ROS2 and which remain on ROS1. Keep node names unchanged when porting.

## Migrated to ROS2
- `logger` (service API stub in `logger` with `logger_ros2_interfaces`; transfer/logging logic still to be ported)
- `logger_binary_node` (service API stub, name preserved)
- `logger_text_node` (service API stub, name preserved)
- `stateControl` (ROS2 Python port in `state_controller`, publishes `state`, serves `get_state`)

## Not Yet Migrated (ROS1)
- All other ROS1 nodes (update this list as you migrate)

## Notes
- Maintain original node names during migration for compatibility.
- Update the tables above as soon as a node is ported or retired.***
