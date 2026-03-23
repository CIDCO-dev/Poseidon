#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export POSEIDON_ROOT="${POSEIDON_ROOT:-${REPO_ROOT}}"
export POSEIDON_E2E_ARTIFACT_DIR="${POSEIDON_E2E_ARTIFACT_DIR:-${SCRIPT_DIR}/artifacts}"
export POSEIDON_HTTP_PORT="${POSEIDON_HTTP_PORT:-8080}"
export POSEIDON_E2E_BASE_URL="${POSEIDON_E2E_BASE_URL:-http://127.0.0.1:${POSEIDON_HTTP_PORT}}"

export DIAGNOSTICS_WS_PORT="${DIAGNOSTICS_WS_PORT:-9099}"
export POSEIDON_TELEMETRY_WS_PORT="${POSEIDON_TELEMETRY_WS_PORT:-9002}"
export POSEIDON_E2E="1"
export POSEIDON_E2E_REUSE_RUNNING="${POSEIDON_E2E_REUSE_RUNNING:-0}"
export POSEIDON_E2E_EXCLUSIVE="${POSEIDON_E2E_EXCLUSIVE:-0}"
export POSEIDON_E2E_LAUNCH_AS_ROOT="${POSEIDON_E2E_LAUNCH_AS_ROOT:-1}"
export POSEIDON_E2E_REQUIRED_NODES="${POSEIDON_E2E_REQUIRED_NODES:-Diagnostics}"
export POSEIDON_E2E_INSTALL_GPSD_CLIENT="${POSEIDON_E2E_INSTALL_GPSD_CLIENT:-1}"

mkdir -p "${POSEIDON_E2E_ARTIFACT_DIR}"

# Best-effort: make ROS tools (rostopic) available for debugging.
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [[ -f "${POSEIDON_ROOT}/src/workspace/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${POSEIDON_ROOT}/src/workspace/devel/setup.bash"
fi

if [[ -z "${POSEIDON_CONFIG_PATH:-}" ]]; then
  export POSEIDON_CONFIG_PATH="${POSEIDON_ROOT}/config.txt"
fi

if [[ -z "${POSEIDON_LOGGER_PATH:-}" ]]; then
  export POSEIDON_LOGGER_PATH="$(mktemp -d -t poseidon-logger-XXXXXX)"
fi
mkdir -p "${POSEIDON_LOGGER_PATH}"

# The E2E now expects a real or externally simulated sonar device.
export POSEIDON_SONAR_DEVICE="${POSEIDON_SONAR_DEVICE:-/dev/sonar}"

HTTP_LOG="${POSEIDON_E2E_ARTIFACT_DIR}/http.log"
SERVICE_LOG="${POSEIDON_E2E_ARTIFACT_DIR}/launchROSService.log"
SERVICE_LAUNCHED_AS_ROOT="0"

if [[ "$(id -u)" -ne 0 ]]; then
  if [[ "${POSEIDON_E2E_LAUNCH_AS_ROOT}" != "1" ]]; then
    echo "[!] E2E requires root for I2C/GPIO access; set POSEIDON_E2E_LAUNCH_AS_ROOT=1."
    exit 1
  fi
  if ! sudo -n true 2>/dev/null; then
    echo "[!] POSEIDON_E2E_LAUNCH_AS_ROOT=1 requires passwordless sudo."
    exit 1
  fi
fi

cleanup() {
  set +e

  if [[ -n "${HTTP_PID:-}" ]] && kill -0 "${HTTP_PID}" 2>/dev/null; then
    kill "${HTTP_PID}" 2>/dev/null || true
    wait "${HTTP_PID}" 2>/dev/null || true
  fi

  if [[ -n "${SERVICE_PID:-}" ]] && kill -0 "${SERVICE_PID}" 2>/dev/null; then
    if [[ "${SERVICE_LAUNCHED_AS_ROOT}" == "1" ]]; then
      sudo kill -INT -- "-${SERVICE_PID}" 2>/dev/null || true
    else
      kill -INT -- "-${SERVICE_PID}" 2>/dev/null || true
    fi
    sleep 8
    if [[ "${SERVICE_LAUNCHED_AS_ROOT}" == "1" ]]; then
      sudo kill -TERM -- "-${SERVICE_PID}" 2>/dev/null || true
    else
      kill -TERM -- "-${SERVICE_PID}" 2>/dev/null || true
    fi
    sleep 3
    if [[ "${SERVICE_LAUNCHED_AS_ROOT}" == "1" ]]; then
      sudo kill -KILL -- "-${SERVICE_PID}" 2>/dev/null || true
    else
      kill -KILL -- "-${SERVICE_PID}" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT INT TERM

run_with_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

integrate_gpsd_client() {
  if [[ "${POSEIDON_E2E_INSTALL_GPSD_CLIENT}" != "1" ]]; then
    echo "[phase 0] skip gpsd_client integration (POSEIDON_E2E_INSTALL_GPSD_CLIENT=${POSEIDON_E2E_INSTALL_GPSD_CLIENT})"
    return 0
  fi

  local workspace_src="${POSEIDON_ROOT}/src/workspace/src"
  local workspace_root="${POSEIDON_ROOT}/src/workspace"
  local gpsd_repo_url="https://github.com/CIDCO-dev/gps_umd.git"
  local gpsd_tmp_dir="/opt/gps_umd"

  if [[ ! -d "${workspace_src}" ]]; then
    echo "[!] Workspace source directory not found: ${workspace_src}"
    exit 1
  fi

  if [[ -d "${workspace_src}/gpsd_client" ]]; then
    echo "[phase 0] gpsd_client already integrated"
    return 0
  fi

  echo "[phase 0] integrate gpsd_client into Poseidon workspace"
  if ! command -v git >/dev/null 2>&1; then
    echo "[!] git is required to install gps_umd."
    exit 1
  fi

  # Equivalent to the field workflow: clone -> move gps* packages -> cleanup.
  run_with_root rm -rf "${gpsd_tmp_dir}" 2>/dev/null || true
  run_with_root git clone "${gpsd_repo_url}" "${gpsd_tmp_dir}"
  run_with_root env GPSD_TMP_DIR="${gpsd_tmp_dir}" WORKSPACE_SRC="${workspace_src}" bash -lc '
    set -e
    shopt -s nullglob
    pkgs=("$GPSD_TMP_DIR"/gps*)
    if [[ ${#pkgs[@]} -eq 0 ]]; then
      echo "[!] No gps* packages found in $GPSD_TMP_DIR." >&2
      exit 1
    fi
    mv "${pkgs[@]}" "$WORKSPACE_SRC/"
  '
  run_with_root rm -rf "${gpsd_tmp_dir}"

  if ! command -v catkin_make >/dev/null 2>&1; then
    echo "[!] catkin_make not found; cannot build workspace after gps_umd install."
    exit 1
  fi

  echo "[phase 0] rebuild workspace after gpsd_client integration"
  pushd "${workspace_root}" >/dev/null
  catkin_make
  popd >/dev/null

  if [[ -f "${POSEIDON_ROOT}/src/workspace/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${POSEIDON_ROOT}/src/workspace/devel/setup.bash"
  fi
}

start_http_server() {
  pushd "${POSEIDON_ROOT}/www/webroot" >/dev/null
  python3 -m http.server "${POSEIDON_HTTP_PORT}" --bind 127.0.0.1 >"${HTTP_LOG}" 2>&1 &
  HTTP_PID=$!
  popd >/dev/null
}

stop_existing_poseidon() {
  if [[ "${POSEIDON_E2E_EXCLUSIVE}" == "1" ]]; then
    # Best-effort cleanup of existing Poseidon instances on a bench.
    if command -v systemctl >/dev/null 2>&1; then
      sudo systemctl stop ros 2>/dev/null || true
    fi
    if command -v fuser >/dev/null 2>&1; then
      sudo fuser -k 9002/tcp 9099/tcp 9003/tcp 9004/tcp 2>/dev/null || true
    fi
  fi
}

start_poseidon_service() {
  if [[ "${POSEIDON_E2E_REUSE_RUNNING}" != "1" ]]; then
    if [[ "${POSEIDON_E2E_LAUNCH_AS_ROOT}" == "1" && "$(id -u)" -ne 0 ]]; then
      setsid sudo -E bash "${POSEIDON_ROOT}/launchROSService.sh" >"${SERVICE_LOG}" 2>&1 &
      SERVICE_LAUNCHED_AS_ROOT="1"
    else
      setsid bash "${POSEIDON_ROOT}/launchROSService.sh" >"${SERVICE_LOG}" 2>&1 &
    fi
    SERVICE_PID=$!
    export POSEIDON_E2E_REUSE_RUNNING="1"
  fi
}

wait_for_ports() {
  if ! python3 - <<'PY'
import os
import socket
import time

host = "127.0.0.1"
ports = [
    int(os.environ.get("DIAGNOSTICS_WS_PORT", "9099")),
    int(os.environ.get("POSEIDON_TELEMETRY_WS_PORT", "9002")),
    int(os.environ.get("POSEIDON_HTTP_PORT", "8080")),
]
deadline = time.time() + int(os.environ.get("POSEIDON_E2E_WAIT_SECONDS", "90"))

pending = set(ports)
last_err = None
while pending and time.time() < deadline:
    for port in list(pending):
        try:
            with socket.create_connection((host, port), timeout=1.0):
                pending.remove(port)
        except OSError as exc:
            last_err = exc
    time.sleep(1.0)

if pending:
    raise SystemExit(f"Timed out waiting for ports: {sorted(pending)} (last error: {last_err})")
PY
  then
    echo "[!] Poseidon did not expose required ports in time."
    if [[ -n "${SERVICE_PID:-}" ]]; then
      echo "[!] launchROSService PID: ${SERVICE_PID}"
    fi
    echo "[!] Tail of ${SERVICE_LOG}:"
    tail -n 200 "${SERVICE_LOG}" || true
    exit 1
  fi
}

wait_for_ros_nodes() {
  if ! command -v rosnode >/dev/null 2>&1; then
    echo "[!] rosnode not found; skipping ROS node wait."
    return 0
  fi

  local deadline
  deadline=$(( $(date +%s) + ${POSEIDON_E2E_WAIT_SECONDS:-90} ))

  local -a required_nodes=()
  if [[ -n "${POSEIDON_E2E_REQUIRED_NODES}" ]]; then
    IFS=',' read -r -a required_nodes <<< "${POSEIDON_E2E_REQUIRED_NODES}"
  fi

  local trimmed
  local -a normalized_required=()
  for node in "${required_nodes[@]}"; do
    trimmed="${node#"${node%%[![:space:]]*}"}"
    trimmed="${trimmed%"${trimmed##*[![:space:]]}"}"
    [[ -z "${trimmed}" ]] && continue
    if [[ "${trimmed}" != /* ]]; then
      trimmed="/${trimmed}"
    fi
    normalized_required+=("${trimmed}")
  done

  while [[ "$(date +%s)" -le "${deadline}" ]]; do
    local nodes
    nodes="$(timeout 3s rosnode list 2>/dev/null || true)"
    if [[ -n "${nodes}" ]]; then
      if [[ "${#normalized_required[@]}" -eq 0 ]]; then
        return 0
      fi

      local missing=()
      for req in "${normalized_required[@]}"; do
        if ! printf '%s\n' "${nodes}" | grep -Fxq "${req}"; then
          missing+=("${req}")
        fi
      done

      if [[ "${#missing[@]}" -eq 0 ]]; then
        return 0
      fi
    fi
    sleep 2
  done

  echo "[!] Timed out waiting for ROS nodes: ${normalized_required[*]:-<any>}"
  if [[ -n "${SERVICE_PID:-}" ]]; then
    echo "[!] launchROSService PID: ${SERVICE_PID}"
  fi
  echo "[!] Tail of ${SERVICE_LOG}:"
  tail -n 200 "${SERVICE_LOG}" || true
  exit 1
}

phase_start_ros() {
  echo "[phase 1] start ROS and wait for nodes"
  start_http_server
  stop_existing_poseidon
  start_poseidon_service
  wait_for_ports
  wait_for_ros_nodes
}

phase_backend_test() {
  echo "[phase 2] run backend websocket test"
  python3 "${POSEIDON_ROOT}/src/workspace/src/diagnostics/tests/test_launchrosservice_websocket.py"
}

phase_ui_test() {
  echo "[phase 3] run UI test"
  pushd "${SCRIPT_DIR}" >/dev/null
  if [[ ! -d node_modules ]]; then
    npm install --silent --no-package-lock
  fi
  set +e
  node ui_test.js
  UI_STATUS=$?
  set -e

  if [[ "${UI_STATUS}" -ne 0 ]]; then
    echo "[!] UI E2E failed with exit code ${UI_STATUS}"
    echo "[!] Tail of ${SERVICE_LOG}:"
    tail -n 200 "${SERVICE_LOG}" || true

    if command -v rostopic >/dev/null 2>&1; then
      echo "[!] rostopic list (first 200):"
      rostopic list 2>/dev/null | head -n 200 || true
      echo "[!] rostopic hz /imu/data (5s):"
      timeout 5s rostopic hz /imu/data || true
      echo "[!] rostopic hz /depth (5s):"
      timeout 5s rostopic hz /depth || true
    else
      echo "[!] rostopic not found; skipping ROS topic debug."
    fi

    python3 - <<'PY' || true
import asyncio
import json
import os

try:
    import websockets
except Exception as exc:
    raise SystemExit(f"websockets not available: {exc}")

PORT = int(os.environ.get("DIAGNOSTICS_WS_PORT", "9099"))
URL = f"ws://127.0.0.1:{PORT}"

async def main():
    async with websockets.connect(URL) as ws:
        await ws.send(json.dumps({"command": "updateDiagnostic"}))
        raw = await ws.recv()
        msg = json.loads(raw)
        diags = {d.get("name"): d for d in msg.get("diagnostics", [])}
        for key in ("GNSS Fix", "IMU Communication", "Sonar Communication"):
            d = diags.get(key)
            if not d:
                print(f"[diagnostics] missing: {key}")
                continue
            print(f"[diagnostics] {key}: status={d.get('status')} message={d.get('message')}")

asyncio.run(main())
PY

    exit "${UI_STATUS}"
  fi
  popd >/dev/null
}

integrate_gpsd_client
phase_start_ros
phase_backend_test
phase_ui_test
