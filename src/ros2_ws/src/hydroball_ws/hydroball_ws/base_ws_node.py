import asyncio
import json
import threading
from typing import Set

import rclpy
from rclpy.node import Node
from websockets import WebSocketServerProtocol, serve


class BaseWsNode(Node):
    """Base WebSocket server node wiring rclpy with an asyncio/websockets server."""

    def __init__(self, name: str, port: int) -> None:
        super().__init__(name)
        self.port = port
        self.loop = asyncio.new_event_loop()
        self.connections: Set[WebSocketServerProtocol] = set()
        self._server = None
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"{name} WebSocket server starting on port {self.port}")

    async def handle_message(self, websocket: WebSocketServerProtocol, message: str):
        """Override in subclasses to handle incoming JSON messages."""
        pass

    async def on_connect(self, websocket: WebSocketServerProtocol):
        pass

    async def on_disconnect(self, websocket: WebSocketServerProtocol):
        pass

    async def _handler(self, websocket: WebSocketServerProtocol, path: str):
        self.connections.add(websocket)
        await self.on_connect(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        finally:
            self.connections.discard(websocket)
            await self.on_disconnect(websocket)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self._server = serve(self._handler, "0.0.0.0", self.port)
        self.loop.run_until_complete(self._server)
        self.loop.run_forever()

    def broadcast(self, payload: str):
        """Send a text payload to all connected clients."""
        async def _send_all():
            to_remove = []
            for ws in self.connections:
                try:
                    await ws.send(payload)
                except Exception as exc:  # pragma: no cover
                    to_remove.append(ws)
                    self.get_logger().warn(f"Failed to send to a client: {exc}")
            for ws in to_remove:
                self.connections.discard(ws)

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(_send_all(), self.loop)

    def stop_server(self):
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def spin(self):
        try:
            rclpy.spin(self)
        finally:
            self.stop_server()
            self.destroy_node()
            rclpy.shutdown()
