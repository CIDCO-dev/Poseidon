# Nodes Overview

This list is generated from `src/workspace/src/*/nodes`. Keep it updated when nodes are added or renamed.

| Package | Node file | Notes |
| --- | --- | --- |
| sonar_dummy | [nodes/sonar_dummy_node.cpp](../src/workspace/src/sonar_dummy/nodes/sonar_dummy_node.cpp) | Dummy sonar node (C++) — see [doc](doc/nodes/sonar_dummy.md) |
| raspberrypi_vitals | [nodes/raspberrypi_vitals_node.cpp](../src/workspace/src/raspberrypi_vitals/nodes/raspberrypi_vitals_node.cpp) | Raspberry Pi vitals publisher (C++) — see [doc](doc/nodes/raspberrypi_vitals.md) |
| imu_null | [nodes/imu_null_node.cpp](../src/workspace/src/imu_null/nodes/imu_null_node.cpp) | IMU null node (C++) — see [doc](doc/nodes/imu_null.md) |
| hydroball_config_websocket | [nodes/hydroball_config_websocket_node.cpp](../src/workspace/src/hydroball_config_websocket/nodes/hydroball_config_websocket_node.cpp) | Hydroball config websocket bridge (C++) — see [doc](doc/nodes/hydroball_config_websocket.md) |
| logger | [nodes/logger_binary_node.cpp](../src/workspace/src/logger/nodes/logger_binary_node.cpp) | Logger binary node (C++) — see [doc](doc/nodes/logger_binary.md) |
| logger | [nodes/logger_text_node.cpp](../src/workspace/src/logger/nodes/logger_text_node.cpp) | Logger text node (C++) — see [doc](doc/nodes/logger_text.md) |
| gnss_dummy | [nodes/gnss_dummy_node.cpp](../src/workspace/src/gnss_dummy/nodes/gnss_dummy_node.cpp) | Dummy GNSS node (C++) — see [doc](doc/nodes/gnss_dummy.md) |
| gnss_zed_f9p | [nodes/gnss_zed_f9p_node.cpp](../src/workspace/src/gnss_zed_f9p/nodes/gnss_zed_f9p_node.cpp) | ZED-F9P GNSS node (C++) — see [doc](doc/nodes/gnss_zed_f9p.md) |
| video_recorder | [nodes/video_recorder_node.cpp](../src/workspace/src/video_recorder/nodes/video_recorder_node.cpp) | Video recorder node (C++) — see [doc](doc/nodes/video_recorder.md) |
| hydroball_data_websocket | [nodes/hydroball_data_websocket_node.cpp](../src/workspace/src/hydroball_data_websocket/nodes/hydroball_data_websocket_node.cpp) | Hydroball data websocket bridge (C++) — see [doc](doc/nodes/hydroball_data_websocket.md) |
| state_controller | [nodes/state_controller_node.cpp](../src/workspace/src/state_controller/nodes/state_controller_node.cpp) | State controller node (C++) — see [doc](doc/nodes/state_controller.md) |
| sonar_nmea_0183_tcp_client | [nodes/network_node.cpp](../src/workspace/src/sonar_nmea_0183_tcp_client/nodes/network_node.cpp) | Sonar NMEA 0183 network client (C++) — see [doc](doc/nodes/sonar_nmea_0183_tcp_client.md) |
| sonar_nmea_0183_tcp_client | [nodes/device_node.cpp](../src/workspace/src/sonar_nmea_0183_tcp_client/nodes/device_node.cpp) | Sonar NMEA 0183 device client (C++) — see [doc](doc/nodes/sonar_nmea_0183_tcp_client.md) |
| diagnostics | [nodes/diagnostics_websocket_node.cpp](../src/workspace/src/diagnostics/nodes/diagnostics_websocket_node.cpp) | Diagnostics websocket bridge (C++) — see [doc](doc/nodes/diagnostics.md) |
| imu_dummy | [nodes/imu_dummy_node.cpp](../src/workspace/src/imu_dummy/nodes/imu_dummy_node.cpp) | Dummy IMU node (C++) — see [doc](doc/nodes/imu_dummy.md) |
| gnss_mosaic_x5 | [nodes/gnss_mosaic_x5_node.cpp](../src/workspace/src/gnss_mosaic_x5/nodes/gnss_mosaic_x5_node.cpp) | GNSS Mosaic X5 node (C++) — see [doc](doc/nodes/gnss_mosaic_x5.md) |
| lidar_filtering | [nodes/lidar_filtering_node.cpp](../src/workspace/src/lidar_filtering/nodes/lidar_filtering_node.cpp) | Lidar filtering node (C++) — see [doc](doc/nodes/lidar_filtering.md) |
| hydroball_files_websocket | [nodes/hydroball_files_websocket_node.cpp](../src/workspace/src/hydroball_files_websocket/nodes/hydroball_files_websocket_node.cpp) | Hydroball files websocket bridge (C++) — see [doc](doc/nodes/hydroball_files_websocket.md) |
| ins_piksi | [nodes/ins_piksi_node.cpp](../src/workspace/src/ins_piksi/nodes/ins_piksi_node.cpp) | INS Piksi node (C++) — see [doc](doc/nodes/ins_piksi.md) |

Per-node docs live in `doc/nodes/*.md` (update as details become available).

When modifying a node, update this table if paths or roles change, and ensure inline comments/docstrings remain accurate.
