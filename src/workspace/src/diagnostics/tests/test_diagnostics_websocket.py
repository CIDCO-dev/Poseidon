#!/usr/bin/env python3

import asyncio
import json
import time
import unittest

try:
    import rospy
    import rostest
except ImportError as exc:  # pragma: no cover - environment guard
    raise unittest.SkipTest(f"ROS Python modules are required: {exc}")

try:
    import websockets
except ImportError:
    websockets = None


WEBSOCKET_URI = "ws://localhost:9099"
UPDATE_COMMAND = json.dumps({"command": "updateDiagnostic"})
CONNECT_TIMEOUT = 20.0
RESPONSE_TIMEOUT = 30.0
RETRY_DELAY = 0.5


@unittest.skipUnless(websockets, "websockets package is required for this integration test")
class DiagnosticsWebsocketTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("diagnostics_websocket_test", anonymous=True)

    async def _request_diagnostics(self):
        deadline = time.time() + CONNECT_TIMEOUT
        last_exception = None

        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                async with websockets.connect(WEBSOCKET_URI) as ws:
                    await ws.send(UPDATE_COMMAND)
                    raw = await asyncio.wait_for(ws.recv(), timeout=RESPONSE_TIMEOUT)
                    return json.loads(raw)
            except Exception as exc:
                last_exception = exc
                await asyncio.sleep(RETRY_DELAY)

        raise AssertionError(f"Unable to retrieve diagnostics via websocket: {last_exception}")

    def test_update_diagnostic_command_returns_payload(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            response = loop.run_until_complete(self._request_diagnostics())
        finally:
            loop.close()

        self.assertIn("diagnostics", response)
        diagnostics = response["diagnostics"]
        self.assertIsInstance(diagnostics, list)
        self.assertGreater(len(diagnostics), 0)

        for entry in diagnostics:
            self.assertIn("status", entry)
            self.assertIn("name", entry)
            self.assertIn("message", entry)


if __name__ == "__main__":
    rostest.rosrun("diagnostics", "diagnostics_websocket_integration", DiagnosticsWebsocketTest)
