#!/usr/bin/env python3

import asyncio
import json
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

try:
    import diagnostics_websocket
    from diagnostics_websocket import (
        ApiConnectionDiagnostic,
        DiagnosticsServer,
        DnsResolutionDiagnostic,
        InternetConnectivityDiagnostic,
    )
    from diagnostic_msgs.msg import DiagnosticStatus
except ImportError:
    diagnostics_websocket = None
    ApiConnectionDiagnostic = None
    DiagnosticsServer = None
    DnsResolutionDiagnostic = None
    InternetConnectivityDiagnostic = None
    DiagnosticStatus = None


class StubWebSocket:
    """Minimal async iterator that captures messages sent by the handler."""

    def __init__(self, messages):
        self._messages = iter(messages)
        self.sent = []

    def __aiter__(self):
        return self

    async def __anext__(self):
        try:
            return next(self._messages)
        except StopIteration:
            raise StopAsyncIteration

    async def send(self, data):
        self.sent.append(data)


class DummyAwaitable:
    def __await__(self):
        async def _coro():
            return self

        return _coro().__await__()


class DummyLoop:
    def __init__(self):
        self.running = False
        self.stopped = False
        self.awaitable = None

    def run_until_complete(self, awaitable):
        self.awaitable = awaitable

    def run_forever(self):
        self.running = True

    def stop(self):
        self.stopped = True
        self.running = False

    def call_soon_threadsafe(self, callback, *args, **kwargs):
        callback(*args, **kwargs)

    def is_running(self):
        return self.running


@unittest.skipIf(DiagnosticsServer is None, "ROS diagnostics modules unavailable")
class DiagnosticsServerUnitTest(unittest.TestCase):
    def setUp(self):
        self.rospy_logs = patch.multiple(
            "diagnostics_websocket.rospy",
            loginfo=MagicMock(),
            logwarn=MagicMock(),
            logerr=MagicMock(),
        )
        self.rospy_logs.start()
        self.addCleanup(self.rospy_logs.stop)

    @staticmethod
    def _status(level, name, message):
        status = DiagnosticStatus()
        status.level = level
        status.name = name
        status.message = message
        return status

    def test_add_test_stores_reference(self):
        server = DiagnosticsServer()
        sentinel = object()

        server.add_test(sentinel)

        self.assertIn(sentinel, server.tests)

    def test_update_command_formats_statuses_and_skips_on_failed_connectivity(self):
        class FailingInternet(InternetConnectivityDiagnostic):
            def __init__(self):
                self.name = "Internet"

            def update(self):
                return DiagnosticsServerUnitTest._status(
                    DiagnosticStatus.ERROR, self.name, "offline"
                )

        class SkippedDns(DnsResolutionDiagnostic):
            def __init__(self):
                self.name = "DNS"

            def update(self):
                return DiagnosticsServerUnitTest._status(
                    DiagnosticStatus.OK, self.name, "should not run"
                )

        class SkippedApi(ApiConnectionDiagnostic):
            def __init__(self):
                self.name = "API"

            def update(self):
                return DiagnosticsServerUnitTest._status(
                    DiagnosticStatus.OK, self.name, "should not run"
                )

        class AlwaysOk:
            def __init__(self):
                self.name = "Other"

            def update(self):
                return DiagnosticsServerUnitTest._status(
                    DiagnosticStatus.OK, self.name, "ok"
                )

        server = DiagnosticsServer()
        server.add_test(FailingInternet())
        server.add_test(SkippedDns())
        server.add_test(SkippedApi())
        server.add_test(AlwaysOk())

        ws = StubWebSocket([json.dumps({"command": "updateDiagnostic"})])
        asyncio.run(server.websocket_handler(ws, path=None))

        self.assertEqual(len(ws.sent), 1)
        payload = json.loads(ws.sent[0])
        diagnostics = payload.get("diagnostics", [])

        self.assertEqual(len(diagnostics), 4)
        self.assertFalse(diagnostics[0]["status"])
        self.assertEqual(diagnostics[1]["message"], "Skipped: internet connectivity failed")
        self.assertEqual(diagnostics[2]["message"], "Skipped: internet connectivity failed")
        self.assertTrue(diagnostics[3]["status"])

    def test_get_running_nodes_aggregates_state(self):
        server = DiagnosticsServer()
        ws = StubWebSocket([json.dumps({"command": "getRunningNodes"})])
        fake_state = (
            [("topic_a", ["node_1", "node_2"])],
            [("topic_b", ["node_2"])],
            [("service_a", ["node_3"])],
        )

        with patch("diagnostics_websocket.rosgraph.masterapi.Master") as master_ctor:
            master = master_ctor.return_value
            master.getSystemState.return_value = fake_state

            asyncio.run(server.websocket_handler(ws, path=None))

        self.assertEqual(len(ws.sent), 1)
        payload = json.loads(ws.sent[0])
        nodes = set(payload.get("running_nodes", []))
        self.assertSetEqual(nodes, {"node_1", "node_2", "node_3"})

    def test_unknown_command_does_not_send_payload(self):
        server = DiagnosticsServer()
        ws = StubWebSocket([json.dumps({"command": "noop"})])

        asyncio.run(server.websocket_handler(ws, path=None))

        self.assertEqual(ws.sent, [])

    def test_start_and_stop_server_use_event_loop(self):
        server = DiagnosticsServer()
        dummy_loop = DummyLoop()
        dummy_server = DummyAwaitable()

        with patch("diagnostics_websocket.asyncio.new_event_loop", return_value=dummy_loop), patch(
            "diagnostics_websocket.websockets.serve", return_value=dummy_server
        ):
            server.start_server(port=9999)

        self.assertIs(server.loop, dummy_loop)
        self.assertIs(server.server, dummy_server)
        self.assertTrue(dummy_loop.running)

        server.stop_server()
        self.assertTrue(dummy_loop.stopped)

    def test_run_in_thread_invokes_start_server(self):
        server = DiagnosticsServer()
        started = threading.Event()

        def fake_start(port):
            if port == 4242:
                started.set()

        with patch.object(server, "start_server", side_effect=fake_start) as start_mock:
            server.run_in_thread(port=4242)

            self.assertTrue(started.wait(timeout=2.0))
            start_mock.assert_called_once_with(4242)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
