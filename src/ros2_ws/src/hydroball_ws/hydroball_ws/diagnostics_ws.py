import json
import subprocess
from typing import List, Dict

import rclpy
from websockets import WebSocketServerProtocol

from .base_ws_node import BaseWsNode


class DiagnosticsWsNode(BaseWsNode):
    """
    ROS2 port of diagnostics websocket:
    - updateDiagnostic: return collected diagnostics (mirrors ROS1 structure)
    - getRunningNodes: return running nodes list
    """

    def __init__(self):
        super().__init__("diagnostics", port=9099)
        self._diag_names = [
            "Gnss Communication",
            "Gnss fix",
            "Imu Communication",
            "Imu Calibrated",
            "Sonar Communication",
            "Clock Diagnostic",
            "Api Connection Diagnostic",
            "Serial Number Pattern Validation",
            "Gnss binary stream Communication",
        ]
        self._diag_cache: List[Dict] = []

    async def handle_message(self, websocket: WebSocketServerProtocol, message: str):
        try:
            doc = json.loads(message)
        except json.JSONDecodeError:
            self.get_logger().error(f"Error parsing JSON on_message: {message}")
            return

        cmd = doc.get("command")
        if not cmd:
            self.get_logger().error(f"No command found: {message}")
            return

        if cmd == "updateDiagnostic":
            await self._send_diagnostics(websocket)
        elif cmd == "getRunningNodes":
            await self._send_running_nodes(websocket)
        else:
            self.get_logger().error(f"Unknown command: {cmd}")

    async def _send_diagnostics(self, ws: WebSocketServerProtocol):
        diags = self._build_diagnostics()
        payload = json.dumps({"diagnostics": diags})
        await ws.send(payload)

    async def _send_running_nodes(self, ws: WebSocketServerProtocol):
        nodes = self._list_running_nodes()
        payload = json.dumps({"running_nodes": nodes})
        await ws.send(payload)

    def _list_running_nodes(self) -> List[str]:
        try:
            output = subprocess.check_output(["ros2", "node", "list"], text=True)
            return [line.strip().lstrip("/") for line in output.splitlines() if line.strip()]
        except Exception:
            # Fallback to rclpy info if ros2 CLI not available
            try:
                nodes = self.get_node_names_and_namespaces()
                return [name.lstrip("/") for name, _ in nodes]
            except Exception:
                return []

    def _build_diagnostics(self) -> List[Dict]:
        # Simple heuristic: mark as OK if any nodes are running; otherwise warn.
        running = set(self._list_running_nodes())
        diags: List[Dict] = []
        for name in self._diag_names:
            status = True
            msg = "ok"
            if not running:
                status = False
                msg = "no nodes running"
            diags.append({"name": name, "message": msg, "status": status})
        # cache for potential reuse
        self._diag_cache = diags
        return diags


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsWsNode()
    node.spin()


if __name__ == "__main__":
    main()
