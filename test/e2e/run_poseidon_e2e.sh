#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export POSEIDON_ROOT="${POSEIDON_ROOT:-${REPO_ROOT}}"
export POSEIDON_E2E_ARTIFACT_DIR="${POSEIDON_E2E_ARTIFACT_DIR:-${SCRIPT_DIR}/artifacts}"
export POSEIDON_HTTP_PORT="${POSEIDON_HTTP_PORT:-8080}"
export POSEIDON_E2E_BASE_URL="${POSEIDON_E2E_BASE_URL:-http://127.0.0.1:${POSEIDON_HTTP_PORT}}"

export DIAGNOSTICS_WS_PORT="${DIAGNOSTICS_WS_PORT:-9099}"
export POSEIDON_TELEMETRY_WS_PORT="${POSEIDON_TELEMETRY_WS_PORT:-9002}"
export POSEIDON_E2E="1"
export POSEIDON_E2E_REUSE_RUNNING="1"

mkdir -p "${POSEIDON_E2E_ARTIFACT_DIR}"

if [[ -z "${POSEIDON_CONFIG_PATH:-}" ]]; then
  export POSEIDON_CONFIG_PATH="${POSEIDON_ROOT}/config.txt"
fi

if [[ -z "${POSEIDON_LOGGER_PATH:-}" ]]; then
  export POSEIDON_LOGGER_PATH="$(mktemp -d -t poseidon-logger-XXXXXX)"
fi
mkdir -p "${POSEIDON_LOGGER_PATH}"

HTTP_LOG="${POSEIDON_E2E_ARTIFACT_DIR}/http.log"
SERVICE_LOG="${POSEIDON_E2E_ARTIFACT_DIR}/launchROSService.log"

cleanup() {
  set +e

  if [[ -n "${HTTP_PID:-}" ]] && kill -0 "${HTTP_PID}" 2>/dev/null; then
    kill "${HTTP_PID}" 2>/dev/null || true
    wait "${HTTP_PID}" 2>/dev/null || true
  fi

  if [[ -n "${SERVICE_PID:-}" ]] && kill -0 "${SERVICE_PID}" 2>/dev/null; then
    kill -INT -- "-${SERVICE_PID}" 2>/dev/null || true
    sleep 8
    kill -TERM -- "-${SERVICE_PID}" 2>/dev/null || true
    sleep 3
    kill -KILL -- "-${SERVICE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

pushd "${POSEIDON_ROOT}/www/webroot" >/dev/null
python3 -m http.server "${POSEIDON_HTTP_PORT}" --bind 127.0.0.1 >"${HTTP_LOG}" 2>&1 &
HTTP_PID=$!
popd >/dev/null

setsid bash "${POSEIDON_ROOT}/launchROSService.sh" >"${SERVICE_LOG}" 2>&1 &
SERVICE_PID=$!

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
  echo "[!] launchROSService PID: ${SERVICE_PID}"
  echo "[!] Tail of ${SERVICE_LOG}:"
  tail -n 200 "${SERVICE_LOG}" || true
  exit 1
fi

python3 "${POSEIDON_ROOT}/src/workspace/src/diagnostics/tests/test_launchrosservice_websocket.py"

pushd "${SCRIPT_DIR}" >/dev/null
if [[ ! -d node_modules ]]; then
  npm install --silent --no-package-lock
fi
node ui_test.js
popd >/dev/null
