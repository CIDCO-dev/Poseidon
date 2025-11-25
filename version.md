# Version History (English)

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
