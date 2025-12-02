#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import glob
import rospy
import subprocess
from typing import Dict

from setting_msg.msg import Setting
from setting_msg.srv import ConfigurationService, ConfigurationServiceRequest

NM_SYS_CONNS_DIR = "/etc/NetworkManager/system-connections/"

def yes_no(flag: bool) -> str:
    return "yes" if flag else "no"

def truthy(s: str) -> bool:
    return str(s).strip().lower() in {"true", "yes", "1", "y"}

class WifiConfigPy:
    def __init__(self):
        # --- ROS init
        self.node_name = "wifi_config"
        rospy.init_node(self.node_name, anonymous=False)

        # Current state
        self.wifi_connections: Dict[str, Dict[str, str]] = {}  # ssid -> {password, autoconnect}
        self.current_ssid: str = ""
        self.current_password: str = ""
        self.autoconnect_status: bool = False
        self.config_callback_counter: int = 0

        # --- Service client (wait for availability)
        rospy.loginfo("Waiting for 'get_configuration' service…")
        rospy.wait_for_service("get_configuration")
        self.cfg_client = rospy.ServiceProxy("get_configuration", ConfigurationService)

        # --- Subscribing to config changes
        self.cfg_sub = rospy.Subscriber(
            "configuration", Setting, self.configuration_callback, queue_size=1000
        )

        # --- Read NM + apply config
        self.get_nmcli_connections()
        self.get_wifi_config()

        rospy.loginfo(
            "Wifi config : wifiTransferEnabled=%s ssid='%s' password='%s'",
            self.autoconnect_status, self.current_ssid, self.current_password
        )

    # -------------------- NM helpers --------------------

    def run(self, cmd: str) -> subprocess.CompletedProcess:
        """Run a shell command and log on failure."""
        try:
            return subprocess.run(
                cmd, shell=True, check=True, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr("Command failed (%s): %s", cmd, e.stderr.strip())
            raise

    def get_auto_connect_status(self) -> None:
        """
        Populate self.wifi_connections[ssid]['autoconnect'] with 'yes'/'no'.
        Split from the end to tolerate SSIDs containing spaces.
        """
        try:
            out = self.run("nmcli -f NAME,AUTOCONNECT con show").stdout.splitlines()
        except Exception:
            return

        if not out:
            return

        # skip header
        for line in out[1:]:
            line = line.strip()
            if not line:
                continue
            # split last token (= AUTOCONNECT)
            parts = line.rsplit(None, 1)
            if len(parts) != 2:
                continue
            ssid, autoconnect = parts[0].strip(), parts[1].strip().lower()
            cfg = self.wifi_connections.setdefault(ssid, {})
            cfg["autoconnect"] = "yes" if autoconnect == "yes" else "no"

    def add_connection_infos(self, path: str) -> None:
        """
        Lightly parse a *.nmconnection file:
            id=..., ssid=..., psk=...
        Same assumption as original C++: id == ssid.
        """
        ssid = password = cid = ""
        try:
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                for raw in f:
                    line = raw.strip()
                    if line.startswith("ssid="):
                        ssid = line[5:].strip()
                    elif line.startswith("psk="):
                        password = line[4:].strip()
                    elif line.startswith("id="):
                        cid = line[3:].strip()
        except Exception as e:
            rospy.logwarn("Failed to open %s: %s", path, e)
            return

        if ssid and cid and ssid == cid:
            self.wifi_connections.setdefault(ssid, {})["password"] = password
        else:
            # same TODO as original C++
            pass

    def get_nmcli_connections(self) -> None:
        # Iterate over persisted connections
        try:
            for nmfile in glob.glob(os.path.join(NM_SYS_CONNS_DIR, "*")):
                self.add_connection_infos(nmfile)
        except Exception as e:
            rospy.logwarn("Failed reading %s: %s", NM_SYS_CONNS_DIR, e)

        # Complete with autoconnect flag
        self.get_auto_connect_status()

    def list_wifi_connection_names(self) -> list:
        """
        Return the Wi-Fi connection names (type 802-11-wireless) known by NM.
        """
        try:
            out = self.run("nmcli -t -f NAME,TYPE con show").stdout.splitlines()
        except Exception:
            return []

        names = []
        for line in out:
            if not line:
                continue
            parts = line.split(":", 1)
            if len(parts) != 2:
                continue
            name, ctype = parts[0].strip(), parts[1].strip()
            if ctype == "802-11-wireless" and name:
                names.append(name)
        return names

    def delete_wifi_connection(self, name: str) -> None:
        try:
            self.run(f"sudo nmcli con delete '{name}'")
        except Exception:
            rospy.logwarn("Impossible de supprimer la connexion Wi‑Fi '%s'", name)

    def purge_wifi_connections(self) -> None:
        """
        Delete all known Wi-Fi connections so only the current config is kept.
        """
        for name in self.list_wifi_connection_names():
            self.delete_wifi_connection(name)
        self.wifi_connections = {}

    def add_connection_to_nmcli(self, ssid: str, password: str, autoconnect: str) -> None:
        cmd = (
            f"nmcli connection add type wifi ssid '{ssid}' "
            f"wifi-sec.key-mgmt wpa-psk wifi-sec.psk '{password}' "
            f"autoconnect {autoconnect} con-name '{ssid}'"
        )
        try:
            self.run(cmd)
        except Exception:
            rospy.logerr("Could not create connection '%s'", ssid)

    def modify_nmcli_connection(self, ssid: str, password: str, autoconnect: str) -> None:
        try:
            self.run(f"sudo nmcli con modify '{ssid}' wifi-sec.psk '{password}' autoconnect {autoconnect}")
            # bounce de la connexion
            if self.run(f"sudo nmcli con down '{ssid}'").returncode == 0:
                self.run(f"sudo nmcli con up '{ssid}'")
        except Exception:
            rospy.logerr("Failed to modify/bounce connection '%s'", ssid)

    # -------------------- Read config via service --------------------

    def get_config_value(self, key: str) -> str:
        req = ConfigurationServiceRequest(key=key)
        try:
            resp = self.cfg_client(req)
            return resp.value
        except Exception as e:
            rospy.logerr("Service get_configuration('%s') failed: %s", key, e)
            return ""

    def get_wifi_config(self) -> None:
        self.current_ssid = self.get_config_value("wifiSSID")
        self.current_password = self.get_config_value("wifiPassword")
        self.autoconnect_status = truthy(self.get_config_value("wifiTransferEnabled"))

        if not self.current_ssid:
            rospy.logwarn("wifiSSID empty: nothing to do for now.")
            return

        # Purge all previous Wi-Fi connections and recreate only the new one
        self.purge_wifi_connections()
        self.add_connection_to_nmcli(
            self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
        )
        self.wifi_connections[self.current_ssid] = {
            "password": self.current_password,
            "autoconnect": yes_no(self.autoconnect_status),
        }

    # -------------------- Callback topic 'configuration' --------------------

    def configuration_callback(self, setting: Setting) -> None:
        key = (setting.key or "").strip()
        val = (setting.value or "").strip()
        # rospy.loginfo("wifi_config: change %s=%s", key, val)

        if key == "wifiSSID" and val:
            self.current_ssid = val
            self.config_callback_counter += 1

        elif key == "wifiPassword" and len(val) >= 8:
            self.current_password = val
            self.config_callback_counter += 1

        elif key == "wifiTransferEnabled":
            self.autoconnect_status = truthy(val)
            self.config_callback_counter += 1

        # Apply once we received the 3 fields (same logic as C++)
        if self.config_callback_counter >= 3:
            # Clean all Wi-Fi connections and recreate only the received one
            self.purge_wifi_connections()
            self.add_connection_to_nmcli(
                self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
            )
            self.wifi_connections[self.current_ssid] = {
                "password": self.current_password,
                "autoconnect": yes_no(self.autoconnect_status),
            }
            self.config_callback_counter = 0

    # -------------------- Run --------------------

    def spin(self):
        rospy.loginfo("wifi_config node started.")
        rospy.spin()


if __name__ == "__main__":
    try:
        WifiConfigPy().spin()
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", e)
