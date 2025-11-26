import json

import rclpy
from websockets import WebSocketServerProtocol

from .base_ws_node import BaseWsNode


class DiagnosticsWsNode(BaseWsNode):
    def __init__(self):
        super().__init__("diagnostics", port=9099)

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
        # ROS1 version gathers multiple diagnostics; here we return an empty list placeholder.
        payload = json.dumps({"diagnostics": []})
        await ws.send(payload)

    async def _send_running_nodes(self, ws: WebSocketServerProtocol):
        nodes = self.get_node_names_and_namespaces()
        running = [name for name, ns in nodes]
        payload = json.dumps({"running_nodes": running})
        await ws.send(payload)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsWsNode()
    node.spin()


if __name__ == "__main__":
    main()
