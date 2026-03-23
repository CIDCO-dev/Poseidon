# Version History (English)

## 2026-02-19
- e2e: removed the internal fake sonar DBT writer from `test/e2e/run_poseidon_e2e.sh`; E2E now expects a real sonar or an external sonar simulator via `POSEIDON_SONAR_DEVICE`.
- e2e: added an automatic pre-test GPSD client integration phase (`POSEIDON_E2E_INSTALL_GPSD_CLIENT=1` by default) that clones `gps_umd`, moves `gps*` packages into `src/workspace/src`, and rebuilds with `catkin_make` when `gpsd_client` is missing.
- docs: updated `docs/test/hardware_e2e.md` to reflect external sonar usage and the new automatic GPSD integration behavior.

## 2026-01-12
- CI: clean Catkin build/devel/logs on self-hosted runners before builds to avoid `.built_by` permission errors.
- CI: pre-clean the GitHub Actions workspace with sudo before checkout to avoid permission errors during repository cleanup.

## 2026-01-08
- e2e: require root launch by default (`POSEIDON_E2E_LAUNCH_AS_ROOT=1`) and reuse the already-started service during the E2E run to avoid duplicate roslaunch.
- diagnostics/e2e: honor `POSEIDON_E2E_LAUNCH_AS_ROOT=1` when the websocket integration test launches `launchROSService.sh`.
- e2e: split the E2E runner into explicit phases (start ROS + wait for nodes, backend test, UI test) with optional required node gating (`POSEIDON_E2E_REQUIRED_NODES`).
- CI: set `POSEIDON_E2E_LAUNCH_AS_ROOT=1` for hardware E2E runs to ensure sensor access.

## 2025-12-18
- e2e: added Playwright-based headless UI smoke test (verifies `diagnostics.html` populates diagnostics + running nodes tables and required sensor diagnostics report OK; optional telemetry check on `index.html`).
- e2e: added unified runner script that starts Poseidon, serves `www/webroot`, waits for ports, runs backend websocket E2E + UI headless checks, and stores artifacts under `test/e2e/artifacts`.
- diagnostics/e2e: gated the launchROSService websocket test behind `POSEIDON_E2E=1` and added `POSEIDON_E2E_REUSE_RUNNING=1` to support CI benches that already run ROS.
- diagnostics: start the WebSocket server from inside the asyncio loop to be compatible with newer `websockets` versions (fixes port 9099 not opening on some systems).
- launch: made `launchROSService.sh` relocatable via `POSEIDON_ROOT` (no longer hard-coded to `/opt/Poseidon`).
- launch: avoid `set -u` in `launchROSService.sh` so sourcing `/opt/ros/noetic/setup.bash` does not fail when ROS scripts reference unset variables (fixes CI E2E startup).
- launch/e2e: pass `loggerPath` and `configPath` from `POSEIDON_LOGGER_PATH` / `POSEIDON_CONFIG_PATH` via `launchROSService.sh` (avoids invalid nested `$(optenv ...)` substitutions in roslaunch args).
- CI: added optional "hardware E2E" workflow steps (Playwright install, run E2E, upload artifacts) triggered on `workflow_dispatch` or pushes to `main/master`.
- diagnostics/e2e: make IMU/sonar communication thresholds configurable via env vars (`POSEIDON_DIAG_IMU_*`, `POSEIDON_DIAG_SONAR_*`) to reduce false negatives on slower benches.
- e2e: allow tuning diagnostics refresh polling delay via `POSEIDON_E2E_DIAGNOSTICS_REFRESH_DELAY_MS` for longer-running diagnostics windows.
- e2e: on UI failure, runner now dumps `roslaunch` tail + basic `rostopic`/diagnostics websocket debug into the job logs for faster triage.
- e2e: avoid `set -u` in the E2E runner when sourcing ROS setup scripts (prevents `ROS_DISTRO: unbound variable` on some environments).
- e2e: keep a DBT NMEA writer running for the whole E2E so `/depth` can be validated when `DIAGNOSTICS_FAKE_SERIAL_PORT` is available.
- e2e: optional exclusive mode (`POSEIDON_E2E_EXCLUSIVE=1`) stops the system `ros` service and frees websocket ports before starting Poseidon to avoid port conflicts on benches.
- launch: pass `sonarDevice` into the Hydrobox launch and map it to `/Sonar/device` so the sonar node can be pointed at a test serial adapter.

## 2025-12-03
- diagnostics: added end-to-end nosetest that launches `launchROSService.sh`, waits for the 9099 diagnostics websocket, sends `updateDiagnostic`, and validates the returned payload; optional DBT feed on `DIAGNOSTICS_FAKE_SERIAL_PORT` (default `/dev/ttyUSB1`) to drive sonar without touching `/dev/sonar`.
- diagnostics: declare `python3-websockets` as exec/test dependency so websocket clients/servers are available during integration runs.
- diagnostics: fixed indentation in the launchROSService websocket test teardown to avoid SyntaxErrors during nosetests.
- diagnostics: guarded `asyncio.set_event_loop` in `diagnostics_websocket.start_server` to prevent dummy-loop `AssertionError` in unit tests.

## 2025-12-02
- Web UI: System Status now shows wlan0 state + SSID using sysfs/proc+iwgetid (no nmcli dependency in telemetry path).
- hydroball_data_websocket: publish Wi‑Fi status/SSID in telemetry payload (`telemetry.wifi`), sourced from `/sys/class/net/wlan0/operstate` and `/proc/net/wireless`.
- wifi_file_transfer_config: remove legacy C++ backups; node now purges old Wi‑Fi connections on config change and documentation/comments converted to English.
- wifi_file_transfer_config: added Python unit tests for wifi_config_node (helpers, nmconnection parsing, autoconnect parsing, purge logic, configuration callback) wired via catkin nosetests.
- Launch: replace missing `diagnostics_node` with `diagnostics_websocket.py` in Hydrobox/Hydroball/Simulator launch files to fix diagnostics startup errors.
- diagnostics: add package-path fallback so diagnostics_websocket imports sibling diagnostic modules when run from install space (fix ModuleNotFoundError on diagnostics_test_base).
- diagnostics: fix `getRunningNodes` handler to unpack publisher/subscriber/service lists correctly (no more "too many values to unpack").
- diagnostics: add DNS resolution check (`google.com`) and Internet connectivity check (`http://example.com`) to the diagnostics suite.
- diagnostics: run Internet connectivity first and skip DNS/API checks when connectivity fails to reduce noisy errors.
- diagnostics: API connection test now uses a configurable timeout (default 2s) to avoid hanging when the API domain is blocked.
- diagnostics: added unit tests for the diagnostics websocket node (commands, node listing, lifecycle helpers) to cover its behaviors without ROS master dependencies.
- diagnostics: install the websocket integration test into the package bin path and skip cleanly when the `websockets` module is missing, so rostest can locate/execute it without false negatives.
- install: rewrote `install/stages/ethernet_move_hotspot_rpi4.sh` with Unix line endings so the bash shebang runs without `/bin/bash^M` errors.
- install: fixed `ethernet_move_hotspot_rpi4.sh` variable handling (no more escaped `$`/awk errors) and robust interface-name replacement in Hotspot.nmconnection.
- launch: All Hydrobox profiles now start `wifi_file_transfer_config_node` so Wi‑Fi provisioning is consistent across RPi/Rock and simulator setups.
- launch: Fixed wifi file transfer node type to `wifi_config_node.py` in all launch profiles/simulator so roslaunch can locate the installed Python node.
- wifi_file_transfer_config: protect hotspot/non-target interfaces (e.g., wlan1) and force new connections onto `wlan0`; skip deleting hotspot profiles when applying Wi‑Fi config.
- wifi_file_transfer_config: refuse to apply Wi‑Fi changes if the hotspot interface (default wlan1) is missing and guard against running when the target interface is absent.
- hydroball_data_websocket: use nmcli to populate Wi‑Fi SSID/state, drop iwgetid dependency, and clear SSID when disconnected.
- Web UI (status): Wi‑Fi row matches other bars (full-width gradient green/red) with SSID + state inline in the header; bar color now follows `wifi.state` up/down.
- Tests: update statusScript Jest stubs to support `.css()` chaining after Wi‑Fi bar changes.

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

# 2025-12-02
- Diagnostics: added unit tests for the diagnostics websocket node (commands, node listing, lifecycle helpers) to cover its behaviors without ROS master dependencies.

## 2025-11-27

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
