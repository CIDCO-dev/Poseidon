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

        # Etat courant
        self.wifi_connections: Dict[str, Dict[str, str]] = {}  # ssid -> {password, autoconnect}
        self.current_ssid: str = ""
        self.current_password: str = ""
        self.autoconnect_status: bool = False
        self.config_callback_counter: int = 0

        # --- Service client (attend l’existence)
        rospy.loginfo("Waiting for 'get_configuration' service…")
        rospy.wait_for_service("get_configuration")
        self.cfg_client = rospy.ServiceProxy("get_configuration", ConfigurationService)

        # --- Subscribing aux changements de config
        self.cfg_sub = rospy.Subscriber(
            "configuration", Setting, self.configuration_callback, queue_size=1000
        )

        # --- Lecture NM + application de la config
        self.get_nmcli_connections()
        self.get_wifi_config()

        rospy.loginfo(
            "Wifi config : wifiTransferEnabled=%s ssid='%s' password='%s'",
            self.autoconnect_status, self.current_ssid, self.current_password
        )

    # -------------------- Utilitaires NM --------------------

    def run(self, cmd: str) -> subprocess.CompletedProcess:
        """Exécute une commande shell et log en cas d’échec."""
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
        Remplit self.wifi_connections[ssid]['autoconnect'] avec 'yes'/'no'.
        On découpe par la fin pour tolérer les SSID avec espaces.
        """
        try:
            out = self.run("nmcli -f NAME,AUTOCONNECT con show").stdout.splitlines()
        except Exception:
            return

        if not out:
            return

        # sauter l’en-tête
        for line in out[1:]:
            line = line.strip()
            if not line:
                continue
            # séparer le dernier token (= AUTOCONNECT)
            parts = line.rsplit(None, 1)
            if len(parts) != 2:
                continue
            ssid, autoconnect = parts[0].strip(), parts[1].strip().lower()
            cfg = self.wifi_connections.setdefault(ssid, {})
            cfg["autoconnect"] = "yes" if autoconnect == "yes" else "no"

    def add_connection_infos(self, path: str) -> None:
        """
        Parse sommairement un fichier *.nmconnection :
            id=..., ssid=..., psk=...
        Hypothèse identique à ton C++ : id == ssid.
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
            rospy.logwarn("Impossible d'ouvrir %s: %s", path, e)
            return

        if ssid and cid and ssid == cid:
            self.wifi_connections.setdefault(ssid, {})["password"] = password
        else:
            # même TODO que le C++ original
            pass

    def get_nmcli_connections(self) -> None:
        # Parcours des connexions persistées
        try:
            for nmfile in glob.glob(os.path.join(NM_SYS_CONNS_DIR, "*")):
                self.add_connection_infos(nmfile)
        except Exception as e:
            rospy.logwarn("Erreur lecture %s: %s", NM_SYS_CONNS_DIR, e)

        # Compléter avec l’auto-connect
        self.get_auto_connect_status()

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

    # -------------------- Lecture config via service --------------------

    def get_config_value(self, key: str) -> str:
        req = ConfigurationServiceRequest(key=key)
        try:
            resp = self.cfg_client(req)
            return resp.value
        except Exception as e:
            rospy.logerr("Service get_configuration('%s') a échoué: %s", key, e)
            return ""

    def get_wifi_config(self) -> None:
        self.current_ssid = self.get_config_value("wifiSSID")
        self.current_password = self.get_config_value("wifiPassword")
        self.autoconnect_status = truthy(self.get_config_value("wifiTransferEnabled"))

        if not self.current_ssid:
            rospy.logwarn("wifiSSID vide: rien à faire pour le moment.")
            return

        # Créer ou modifier
        cfg = self.wifi_connections.get(self.current_ssid)
        if cfg is None:
            self.wifi_connections[self.current_ssid] = {
                "password": self.current_password,
                "autoconnect": yes_no(self.autoconnect_status),
            }
            self.add_connection_to_nmcli(
                self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
            )
        else:
            self.modify_nmcli_connection(
                self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
            )

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

        # Appliquer quand on a reçu les 3 champs (même logique que le C++)
        if self.config_callback_counter >= 3:
            if self.current_ssid not in self.wifi_connections:
                self.wifi_connections[self.current_ssid] = {
                    "password": self.current_password,
                    "autoconnect": yes_no(self.autoconnect_status),
                }
                self.add_connection_to_nmcli(
                    self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
                )
            else:
                self.modify_nmcli_connection(
                    self.current_ssid, self.current_password, yes_no(self.autoconnect_status)
                )
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
