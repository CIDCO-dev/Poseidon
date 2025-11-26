import subprocess
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node

from setting_msg.msg import Setting
from setting_msg.srv import ConfigurationService
from .utils import trim_spaces


class WifiConfigNode(Node):
    """
    ROS2 port of wifi_file_transfer_config:
    - listens to configuration updates (wifiSSID/wifiPassword/wifiTransferEnabled)
    - adds/modifies nmcli connections, sets DNS, autoconnect, and activates wlan0
    - stores known connections map
    """

    def __init__(self):
        super().__init__("wifi_file_transfer_config")
        self.configuration: Dict[str, Dict[str, str]] = {}
        self.current_ssid = ""
        self.current_password = ""
        self.autoconnect = False
        self.config_count = 0

        self.config_client = self.create_client(ConfigurationService, "get_configuration")
        self.config_sub = self.create_subscription(Setting, "configuration", self.configuration_callback, 10)

        self._load_existing_connections()
        self._load_initial_config()

    # --- Config handling ---
    def _load_initial_config(self):
        if not self.config_client.wait_for_service(timeout_sec=1.0):
            return
        for key in ("wifiSSID", "wifiPassword", "wifiTransferEnabled"):
            req = ConfigurationService.Request()
            req.key = key
            future = self.config_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result():
                val = future.result().value
                self._apply_config_key(key, val)
        # Apply if we got enough info
        if self.current_ssid and self.current_password:
            self._ensure_connection()

    def configuration_callback(self, setting: Setting):
        self._apply_config_key(setting.key, setting.value)
        if self.config_count >= 3 and self.current_ssid and self.current_password:
            self._ensure_connection()

    def _apply_config_key(self, key: str, value: str):
        if key == "wifiSSID":
            self.config_count += 1
            if value:
                self.current_ssid = value
        elif key == "wifiPassword":
            self.config_count += 1
            if len(value) >= 8:
                self.current_password = value
        elif key == "wifiTransferEnabled":
            self.config_count += 1
            self.autoconnect = value.lower() in ("true", "yes", "1", "on")

    # --- nmcli helpers ---
    def _load_existing_connections(self):
        try:
            for path in Path("/etc/NetworkManager/system-connections/").iterdir():
                if not path.is_file():
                    continue
                ssid = ""
                password = ""
                conn_id = ""
                for line in path.read_text().splitlines():
                    line = trim_spaces(line)
                    if line.startswith("ssid="):
                        ssid = trim_spaces(line[5:])
                    elif line.startswith("psk="):
                        password = trim_spaces(line[4:])
                    elif line.startswith("id="):
                        conn_id = trim_spaces(line[3:])
                if ssid and conn_id == ssid:
                    cfg = self.configuration.setdefault(ssid, {})
                    cfg["password"] = password
            # autoconnect status
            self._fetch_autoconnect_status()
        except Exception as exc:
            self.get_logger().warn(f"Failed to load existing connections: {exc}")

    def _fetch_autoconnect_status(self):
        try:
            result = subprocess.check_output(["nmcli", "-f", "NAME,AUTOCONNECT", "con", "show"], text=True)
            for line in result.splitlines()[1:]:
                parts = line.split()
                if len(parts) >= 2:
                    ssid = parts[0]
                    auto = parts[1]
                    cfg = self.configuration.setdefault(ssid, {})
                    cfg["autoconnect"] = auto
        except Exception as exc:
            self.get_logger().warn(f"Failed autoconnect fetch: {exc}")

    def _run_cmd(self, cmd: str):
        try:
            self.get_logger().info(f"Running: {cmd}")
            subprocess.check_call(cmd, shell=True)
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(f"Command failed ({exc.returncode}): {cmd}")

    def _ensure_connection(self):
        ssid = self.current_ssid
        password = self.current_password
        auto = "yes" if self.autoconnect else "no"
        cfg = self.configuration.get(ssid)
        if cfg is None:
            # add new connection
            self.configuration[ssid] = {"password": password, "autoconnect": auto}
            cmd = (
                f"nmcli connection add type wifi ifname wlan0 ssid \"{ssid}\" "
                f"con-name \"{ssid}\" wifi-sec.key-mgmt wpa-psk wifi-sec.psk \"{password}\" autoconnect {auto}"
            )
            self._run_cmd(cmd)
        else:
            # modify existing
            cmd = f"nmcli con modify \"{ssid}\" wifi-sec.psk \"{password}\" autoconnect {auto}"
            self._run_cmd(cmd)
        # set DNS and assign interface, bring up
        self._run_cmd(f"nmcli connection modify \"{ssid}\" ipv4.dns \"8.8.8.8\"")
        self._run_cmd(f"nmcli connection modify \"{ssid}\" ifname wlan0")
        self._run_cmd(f"nmcli connection down \"{ssid}\" || true")
        self._run_cmd(f"nmcli connection up \"{ssid}\"")


def main(args=None):
    rclpy.init(args=args)
    node = WifiConfigNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
