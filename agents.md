# Agents.md — Contributor and Operator Guide (Poseidon)

This guide targets contributors (humans or agents) working in Poseidon. It covers conventions, structure, build/test procedures, and embedded ROS specifics.

## Goals
- Documentation policy: English only across the repository.
- Track ROS1 changes in `version.md`; keep it up to date and group entries by date.
- Make minimal, safe, targeted changes.
- Respect Catkin structure and separation of C++/Python/launch.
- Keep compatibility across embedded targets (RPi/RockPi) and VM.

## Tech Context
- ROS: Noetic (Ubuntu 20.04)
- Build: Catkin (`src/workspace`), CMake, g++
- Languages: C++ (ROS nodes), Python (scripts/nodes/tools), HTML/JS/CSS (web UI)
- Services: systemd `ros` executes `launchROSService.sh`
- GPSD: system service, port 2947 by default

## Working Layout
- Catkin workspace: `src/workspace`
  - Packages: `src/workspace/src/<package>`
  - Launch files: `src/workspace/launch/...`
- Web UI: `www/webroot`
- Install scripts: `install/stages/*.sh`
- Global config: `config.txt`
- Documentation: keep source in `doc/` as Markdown; CI can generate PDFs for download, but `doc/*.md` remain the source of truth (viewable directly in the repo).

## Local Build
1) Source ROS: `source /opt/ros/noetic/setup.bash`
2) Build workspace:
   - `cd src/workspace`
   - VM/CI: `catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES="mavros;...;inertial_sense;lidar_filtering"`
   - Embedded: adjust blacklist for RAM/CPU as needed.
3) Source devel: `source devel/setup.bash`

## Run Quickstart
- VM simulation: `roslaunch launch/Simulator/dummy_simulator_virtual-machine.launch`
- Embedded profiles: see `launch/Hydrobox/*`, `launch/Echoboat/*`.
- Boot service: `sudo systemctl start|stop|status ros`

## Add/Modify a ROS Package
- Location: `src/workspace/src/<new_package>`
- Minimum files: `package.xml`, `CMakeLists.txt`, optionally `nodes/` (scripts), `src/` (C++), `include/` (headers)
- Dependencies: declare build/exec/test correctly in `package.xml`.
- C++: follow ROS idioms (`ros::NodeHandle`, params, `rosparam`).
- Python: PEP 8; use `#!/usr/bin/env python3` and executable perms for `nodes/`.
- Tests: prefer `rostest`/`gtest` (see Jenkins/JUnit patterns).

## Launch Files
- Place under `src/workspace/launch/<Domain>/`.
- Expose clear `arg`s (`loggerPath`, `configPath`, ports, devices) and well-named `param`s.
- Prefer `respawn="true"` and `respawn_delay` for critical nodes.

## Config and Secrets
- `config.txt` holds runtime config (API, geofence, offsets, sonar, Wi‑Fi). Do not commit real secrets.
- Honor `configPath` in launch files and WebSocket nodes.

## Web UI
- Static assets under `www/webroot` (SB Admin 2, MIT).
- Avoid runtime external dependencies (embedded context).
- Data/config flows over `hydroball_*_websocket` nodes.

## Tools and Scripts
- Rosbags: `tools/ROSBag-Eample/*`, `tools/Edit-Rosbag/*` (extract, filter, count, drop topics).
- Georeferencing: `tools/Georef_script/*` and `logger/lidarGeoreferencer` (post-build).
- GNSS ZED‑F9P: `tools/GNSS-Config-Files/ZED-F9P/*` (stop/start `gpsd`/`ros` around ops).

## Tests & CI
- Local: `catkin_make run_tests -j1` (adjust blacklist as needed).
- Jenkins: see `Jenkinsfile` and `test/scripts/*`.
- JUnit outputs: `build/test_results/.../*.xml` (e.g., angles/gtest-utest.xml).

## Quality & Style
- C++: keep formatting consistent; avoid needless allocations in hot paths.
- Python: PEP 8, clear logging, exceptions handled.
- Launch: explicit node names; document `args`/`params`.
- Messages/Topics: stable names; document breaking changes in the manifest.

## Performance & Robustness
- Use `respawn`/`respawn_delay` for critical nodes.
- On embedded, avoid large message copies; prefer const& in C++.
- For rosbag/SLAM, document prerequisites (IMU covariances, topics) — see `cartographer.md`.

## Security
- No real keys in repo (`config.txt` uses placeholders).
- Do not expose services on WAN by default; prefer LAN for the web UI.

## Review & PR Checklist
- [ ] Catkin build OK (VM or target)
- [ ] Relevant unit tests added/updated
- [ ] Launch files and parameters documented
- [ ] Inline code comments/docstrings remain accurate
- [ ] Related docs in `doc/` updated when behavior/usage changes
- [ ] `version.md` updated with notable changes
- [ ] No secrets or non-portable hardcoded paths
- [ ] `manifest.md` updated for structural changes
- [ ] Run `clang-format` and `clang-tidy` for C++ changes (style/diagnostics)
- [ ] Run `black` (and applicable linters) for Python changes
