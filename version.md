# Version History (English)

## 2025-12-01
- diagnostics: added rostest that launches the WebSocket node and validates `updateDiagnostic` response payload (catkin test friendly).
- raspberrypi_vitals: declare logger_service and power_management_msg dependencies so ToggleLogging and battery message headers exist at build time.
- diagnostics: install the websocket integration test script via catkin so rostest can locate/execute it.
- ROS tests: add pytest skip guards for ROS-dependent Python tests to avoid failures when ROS Python modules are absent.

## 2025-11-27
- CI: gcovr step split into separate XML/HTML runs (no more skipped HTML output) and coverage summary script now falls back to lcov/HTML when XML is absent.
- CI: coverage summary now also falls back to JS lcov when coverage-summary.json is missing (ensures JS coverage is reported).
- CI: coverage summary surfaces JUnit test totals/errors/failures and lists failing suites (links to junit-test-results artifact).
- JS: VM-based Jest tests now instrument scripts before execution so coverage is reported correctly (no longer 0%).
- raspberrypi_vitals: ensure targets depend on generated messages (catkin_EXPORTED_TARGETS) and declare i2c_controller_service in CATKIN_DEPENDS to fix missing `raspberrypi_vitals_msg/sysinfo.h` during build.
- gnss_zed_f9p: declare binary_stream_msg/virtual_serial_port deps and depend on catkin_EXPORTED_TARGETS so generated headers (e.g., Stream.h) exist before building.
- lidar_filtering: fixed tests by passing min/max angle/distance as const refs in Filters helpers (no more rvalue binding errors).
- CI: junit summary lines are now highlighted in red when errors/failures/skipped are non-zero for easier scanning.
- Docs: agent guidance updated to keep Markdown docs in `doc/` (source of truth) with CI able to generate PDFs for download.
- Docs: README and doc/README now target RPi4 on Ubuntu 20.04, using install/rpi4-noetic.sh as the install path.
- Docs: added `doc/install.md` detailing prerequisites and install steps for RPi4 Ubuntu 20.04.
- Docs: added `doc/nodes.md` (ROS nodes list) and `doc/web.md` (web UI pages list); README links updated.
- Docs: added per-web-page stubs in `doc/web/*.md` (index, status, diagnostics, data, map, settings, calibration, goal).
- Docs: node/web index now link to specific Markdown docs and source files for easier navigation.
- Docs: documented `sonar_dummy_node` (overview, published topic, config service keys, behavior).
- Docs: documented `raspberrypi_vitals_node` (vitals topic fields, I2C service calls, thresholds).
- Docs: documented `imu_null_node` (zeroed IMU publisher at 200 Hz).
- Docs: documented `hydroball_config_websocket_node` (websocket config server, services, config file handling, IMU transform broadcast).
- Docs: documented `logger_binary_node` (binary logging of GNSS/IMU/sonar/lidar/vitals/speed, rotation/transfer).
- Docs: documented `logger_text_node` (per-stream text logs, rotation/transfer).
- Docs: documented `gnss_dummy_node` (synthetic NavSatFix publisher with alternating status/path).
- Docs: documented `gnss_zed_f9p_node` (serial UBX ingest, speed/diag/raw stream).
- Docs: documented `video_recorder_node` (camera capture → EXIF-tagged PNGs with interpolated GNSS).
- Docs: documented `hydroball_data_websocket_node` (WebSocket telemetry/recording control on port 9002).
- Docs: documented `state_controller_node` (aggregates fix/IMU/depth/vitals into state + get_state service).
- Docs: documented `sonar_nmea_0183_tcp_client` (network + serial NMEA readers publishing depth/fix/speed).
- Docs: documented `diagnostics_websocket_node` (WebSocket diagnostics/ running-nodes commands on port 9099).
- Docs: documented `imu_dummy_node` (synthetic IMU on imu/data at 200 Hz).
- Docs: documented `gnss_mosaic_x5_node` (raw SBF logger; waits for fix, reads serial, writes .sbf).
- Docs: documented `lidar_filtering_node` (angle/distance pointcloud filter).
- Docs: documented `hydroball_files_websocket_node` (WebSocket file listing/deletion/transfer control on port 9003).

## 2025-11-26
- Install scripts: added shared chrony update script (PPS/NMEA + pool.ntp.org); lighttpd now points to `/opt/Poseidon/www/webroot`.
- Logger: removed warning spam when toggle requested without GPS fix (ROS1).
- CI: GitHub Actions now posts C++/Python/JS coverage summary to the workflow summary.
- Logger: transfer loop now counts zip files once, reports status, sends, and removes on success; RapidJSON string creation simplified.
- CI: Fixed YAML indentation for coverage summary step (workflow now parses correctly).
- CI: Coverage summary heredoc anchored with indentation (python block).
- CI: Coverage summary heredoc flattened (no leading spaces) to avoid Python IndentationError.
- CI: Coverage summary now runs from a separate script `.github/workflows/scripts/coverage_summary.py`.
- Build fix: added `binary_stream_msg` dependency to diagnostics package (resolves missing `binary_stream_msg/Stream.h` include).

## 2025-11-25
- Added HTTPS transfer integration tests for logger binary and text nodes (local TLS server, payload verification).
- Added memory-resilience tests: repeated failed transfers must not increase RSS.
- Fixed `create_json_str` to avoid allocation leak (use `SetString` without manual `new`).
- Updated agent guidelines to maintain this `version.md`.
- Updated agent guidelines: C++ formatting/lint (`clang-format`, `clang-tidy`) now tracked in the Review & PR checklist.
- Updated agent guidelines: Python formatting (`black`) added to the Review & PR checklist.
- CI: JavaScript tests step now targets `www/webroot/js` (npm ci/install + npm test if package.json present).
- Added Jest setup in `www/webroot/js` with a syntax smoke test for all JS files.
- Added Jest tests for dashboardscript.js and script.js (basic DOM/jQuery stubs, helper behavior checks).
- Added Jest test stub for poseidon.js (verifies WebSocketInit is defined).
- Added Jest tests for statusScript.js (processState handling of uptime/humidity and missing telemetry).
- Added Jest tests for settingsScript.js (rendering config and saveConfig payload).
- Added Jest tests for recordingScript.js and mapScript.js (stubs for WebSocket/Leaflet; basic behavior checks).
- Added Jest tests for diagnosticsScript.js (table population for diagnostics and running nodes).
- Added Jest tests for dataScript.js (state processing, publish status UI updates).
- Added Jest test for calibrationScript.js (zeroImu command dispatch).
- JavaScript: npm test now runs Jest with coverage (collects coverage excluding minified/vendor).
- Fixed logger text test build by defining `currentRssBytes` helper in `test_logger_text_node.cpp`.
- CI: add C++ coverage via gcovr, Python coverage via coverage/pytest, JS coverage artifact upload; switched C++ build to `--coverage`.
- CI: install Node 18 with setup-node to satisfy Jest engine requirements for JS tests.
- JS: add guards/stubs for WebSocket/jQuery/Leaflet in browser scripts to allow Jest to run headless, and exclude node_modules/coverage from syntax scan.
- JS: added Jest jsdom setup with global stubs (window/document/$/WebSocket), guarded polling, and map/WKT fallbacks to stabilize headless tests and avoid npm ci lockfile errors.
- JS tests: added explicit `jest-environment-jsdom` devDependency to satisfy Jest config.
- JS housekeeping: generated package-lock for reproducible installs and removed committed coverage output; keep coverage artifacts untracked.
- hydroball_config_websocket: add message deps (geometry/state/setting) to catkin/package.xml and enforce build order on message targets to fix missing Setting.h during catkin_make.
- Logger tests: removed embedded PEM key/cert; tests now generate ephemeral self-signed TLS material at runtime to avoid shipping secrets.
- sonar_nmea_0183_tcp_client: use ROS params for IP/port (instead of argv) and add socket receive timeout to reconnect if the network stream stalls.
- Install scripts: added shared chrony update script (PPS/NMEA + pool.ntp.org); lighttpd now points to `/opt/Poseidon/www/webroot`.
