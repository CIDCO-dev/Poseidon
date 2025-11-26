import asyncio
import json
import subprocess
import threading
from pathlib import Path
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from websockets import WebSocketServerProtocol

from logger_interfaces.srv import TriggerTransfer

from .base_ws_node import BaseWsNode


class FilesWsNode(BaseWsNode):
    def __init__(self):
        super().__init__("hydroball_files_websocket", port=9003)
        self.log_path = Path(
            self.declare_parameter("log_path", "/opt/Poseidon/www/webroot/record").value
        )
        self.config_path = Path(
            self.declare_parameter("config_path", "/opt/Poseidon/config.txt").value
        )
        self.trigger_transfer = self.create_client(TriggerTransfer, "trigger_transfer")

    async def handle_message(self, websocket: WebSocketServerProtocol, message: str):
        try:
            doc = json.loads(message)
        except json.JSONDecodeError:
            self.get_logger().error(f"Error parsing JSON on_message: {message}")
            return

        if "delete" in doc:
            file_name = doc["delete"]
            if isinstance(file_name, str) and file_name.startswith("record/"):
                file_name = file_name.split("/", 1)[1]
            file_to_delete = self.log_path / file_name
            self.delete_file(file_to_delete)
        elif "f-list" in doc:
            await self.send_file_list(websocket)
        elif "publishfiles" in doc:
            threading.Thread(target=self._handle_publish_files, args=(websocket,), daemon=True).start()
        else:
            self.get_logger().error("No command found")

    async def on_connect(self, websocket: WebSocketServerProtocol):
        # Send the initial file list on connection
        await self.send_file_list(websocket)

    async def on_disconnect(self, websocket: WebSocketServerProtocol):
        return

    def delete_file(self, path: Path):
        try:
            path.unlink()
            self.get_logger().info(f"File successfully deleted: {path}")
        except Exception as exc:
            self.get_logger().error(f"Error deleting file {path}: {exc}")

    def get_files_to_transfer(self) -> List[Path]:
        if not self.log_path.exists():
            return []
        return [p for p in self.log_path.iterdir() if p.is_file()]

    async def send_file_list(self, ws: WebSocketServerProtocol):
        fileslist = []
        for p in self.get_files_to_transfer():
            fileslist.append([p.name, f"record/{p.name}"])
        payload = json.dumps({"fileslist": fileslist})
        await ws.send(payload)

    def _handle_publish_files(self, ws: WebSocketServerProtocol):
        try:
            self._send_status(ws, "Checking files to transfer...")
            asyncio.run_coroutine_threadsafe(asyncio.sleep(1.0), self.loop).result()

            files = self.get_files_to_transfer()
            if not files:
                self._send_status(ws, "❌ No files to transfer", done=True)
                return

            self._send_status(ws, "Checking internet connection...")
            asyncio.run_coroutine_threadsafe(asyncio.sleep(1.0), self.loop).result()
            if not self.is_internet_available():
                self._send_status(ws, "❌ No internet connection", done=True)
                return

            self._send_status(ws, "Checking API server connection...")
            asyncio.run_coroutine_threadsafe(asyncio.sleep(1.0), self.loop).result()
            if not self.is_api_available():
                self._send_status(ws, "❌ No API server connection", done=True)
                return

            self._send_status(ws, "Starting files transfer...")
            if not self.trigger_transfer.wait_for_service(timeout_sec=2.0):
                self._send_status(ws, "❌ trigger_transfer service unavailable", done=True)
                return
            req = TriggerTransfer.Request()
            future = self.trigger_transfer.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.done() and future.result():
                res = future.result()
                status_msg = f"✅ Transfer done : {res.message}" if res.success else f"❌ Transfer failed : {res.message}"
                self._send_status(ws, status_msg, done=True)
            else:
                self._send_status(ws, "❌ Fail to start the file transfer", done=True)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Error in file transfer: {exc}")
            self._send_status(ws, "❌ Error during file transfer", done=True)

    def _send_status(self, ws: WebSocketServerProtocol, message: str, done: bool = False):
        doc = {"publishstatus": message}
        if done:
            doc["done"] = True
        payload = json.dumps(doc)
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(ws.send(payload), self.loop)

    def get_api_server_address(self) -> str:
        if not self.config_path.exists():
            self.get_logger().error(f"Fail to open {self.config_path}")
            return ""
        try:
            with self.config_path.open() as f:
                for line in f:
                    if line.startswith("apiServer"):
                        value = line[len("apiServer") :].strip()
                        if not value or value == "localhost":
                            self.get_logger().error("apiServer empty or localhost")
                            return ""
                        return value
        except Exception as exc:
            self.get_logger().error(f"Error reading config: {exc}")
        self.get_logger().error("key 'apiServer' not found in config")
        return ""

    def ping_address(self, addr: str) -> bool:
        return subprocess.call(["ping", "-c", "1", "-W", "1", addr], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0

    def is_internet_available(self) -> bool:
        return self.ping_address("8.8.8.8")

    def is_api_available(self) -> bool:
        domain = self.get_api_server_address()
        if not domain or domain == "localhost":
            return False
        import urllib.request
        try:
            req = urllib.request.Request(f"https://{domain}/", method="HEAD")
            with urllib.request.urlopen(req, timeout=2) as resp:
                return 200 <= resp.status < 400
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = FilesWsNode()
    node.spin()


if __name__ == "__main__":
    main()
