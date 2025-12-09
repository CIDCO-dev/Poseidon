#!/usr/bin/env python3

import asyncio
import json
import os
import signal
import subprocess
import time
import unittest
from pathlib import Path
import threading

import websockets


DEFAULT_PORT = int(os.environ.get("DIAGNOSTICS_WS_PORT", "9099"))
LAUNCH_TIMEOUT = int(os.environ.get("DIAGNOSTICS_WS_LAUNCH_TIMEOUT", "60"))
RESPONSE_TIMEOUT = int(os.environ.get("DIAGNOSTICS_WS_RESPONSE_TIMEOUT", "45"))
FAKE_SERIAL_PORT = os.environ.get("DIAGNOSTICS_FAKE_SERIAL_PORT", "/dev/ttyUSB1")


def find_launch_script():
    """Look up launchROSService.sh starting from this test directory and walking upwards."""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "launchROSService.sh"
        if candidate.is_file():
            return candidate
    return None


async def wait_for_websocket(port, timeout):
    """Wait until the diagnostics websocket accepts connections."""
    url = f"ws://localhost:{port}"
    deadline = time.time() + timeout
    last_error = None

    while time.time() < deadline:
        try:
            async with websockets.connect(url):
                return
        except Exception as exc:  # noqa: BLE001 - best-effort connection loop
            last_error = exc
            await asyncio.sleep(1.0)

    raise TimeoutError(f"WebSocket server not reachable at {url}: {last_error}")


async def request_diagnostics(port):
    """Send the updateDiagnostic command and return the parsed payload."""
    url = f"ws://localhost:{port}"
    async with websockets.connect(url) as ws:
        await ws.send(json.dumps({"command": "updateDiagnostic"}))
        message = await asyncio.wait_for(ws.recv(), timeout=RESPONSE_TIMEOUT)
        return json.loads(message)


class LaunchRosServiceWebsocketTest(unittest.TestCase):
    """Integration test: start launchROSService.sh and verify diagnostics websocket replies."""

    process = None
    launch_script = None

    @classmethod
    def setUpClass(cls):
        # Allow overriding the script location for installed vs. source trees.
        env_override = os.environ.get("POSEIDON_LAUNCH_SCRIPT")
        if env_override:
            cls.launch_script = Path(env_override)
        else:
            cls.launch_script = find_launch_script()

        if not cls.launch_script or not cls.launch_script.is_file():
            raise unittest.SkipTest("launchROSService.sh not found; cannot run integration test.")

        cls.fake_serial_stop = cls._maybe_start_fake_serial_writer()

        cls.process = subprocess.Popen(
            ["bash", str(cls.launch_script)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

        try:
            asyncio.run(wait_for_websocket(DEFAULT_PORT, LAUNCH_TIMEOUT))
        except Exception:
            cls._terminate_process()
            raise

    @classmethod
    def tearDownClass(cls):
        cls._terminate_process()

    @classmethod
    def _terminate_process(cls):
        if hasattr(cls, "fake_serial_stop") and cls.fake_serial_stop:
            cls.fake_serial_stop.set()

        if cls.process and cls.process.poll() is None:
            try:
                os.killpg(os.getpgid(cls.process.pid), signal.SIGINT)
            except ProcessLookupError:
                return

            try:
            cls.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(cls.process.pid), signal.SIGTERM)
            cls.process.wait(timeout=10)

    @staticmethod
    def _maybe_start_fake_serial_writer():
        """
        Some test benches need a NMEA0183 DBT feed at ~1 Hz on a USB serial port that is NOT /dev/sonar.
        If the port exists and is writable, launch a background writer so sonar diagnostics can consume data.
        """
        if not FAKE_SERIAL_PORT:
            return None

        port_path = Path(FAKE_SERIAL_PORT)
        if not port_path.exists():
            return None

        stop_event = threading.Event()

        def compute_checksum(payload: str) -> str:
            checksum = 0
            for ch in payload:
                checksum ^= ord(ch)
            return f"{checksum:02X}"

        def writer():
            payload = "SDDBT,30.9,f,9.4,M,5.1,F"
            sentence = f"${payload}*{compute_checksum(payload)}\r\n"
            while not stop_event.is_set():
                try:
                    with open(port_path, "wb", buffering=0) as fd:
                        fd.write(sentence.encode("ascii"))
                except Exception:
                    # Best effort: if the port disappears, stop quietly.
                    break
                stop_event.wait(1.0)

        threading.Thread(target=writer, daemon=True).start()
        return stop_event

    def test_diagnostics_websocket_returns_expected_entries(self):
        payload = asyncio.run(request_diagnostics(DEFAULT_PORT))
        diagnostics = payload.get("diagnostics", [])

        self.assertTrue(diagnostics, "No diagnostics payload returned from websocket.")

        expected_names = {
            "Internet Connectivity",
            "DNS Resolution",
            "API Connection",
            "BinaryStreamGnss",
            "Clock Diagnostic",
            "GNSS Communication",
            "GNSS Fix",
            "IMU Calibrated",
            "IMU Communication",
            "Serial Number Pattern Validation",
            "Sonar Communication",
        }

        received_names = {entry.get("name") for entry in diagnostics}

        missing = expected_names - received_names
        self.assertFalse(missing, f"Missing diagnostics entries: {sorted(missing)}")

        for entry in diagnostics:
            self.assertIn("status", entry)
            self.assertIn("name", entry)
            self.assertIn("message", entry)
            self.assertIsInstance(entry["status"], bool)
            self.assertIsInstance(entry["name"], str)
            self.assertIsInstance(entry["message"], str)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
