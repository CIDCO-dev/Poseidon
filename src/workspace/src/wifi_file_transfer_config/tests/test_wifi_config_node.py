import os
import tempfile
import subprocess
import types
import unittest
from unittest.mock import MagicMock, patch


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
if SCRIPT_DIR not in os.sys.path:
    os.sys.path.append(SCRIPT_DIR)

import wifi_config_node  # noqa: E402


class WifiConfigNodeTests(unittest.TestCase):
    def make_instance(self):
        """Build a WifiConfigPy instance without running __init__ (avoid ROS side effects)."""
        inst = wifi_config_node.WifiConfigPy.__new__(wifi_config_node.WifiConfigPy)
        inst.wifi_connections = {}
        inst.current_ssid = ""
        inst.current_password = ""
        inst.autoconnect_status = False
        inst.config_callback_counter = 0
        return inst

    def test_truthy_and_yes_no_helpers(self):
        self.assertTrue(wifi_config_node.truthy("Yes"))
        self.assertFalse(wifi_config_node.truthy("no"))
        self.assertEqual("yes", wifi_config_node.yes_no(True))
        self.assertEqual("no", wifi_config_node.yes_no(False))

    def test_get_auto_connect_status_parses_nmcli(self):
        inst = self.make_instance()
        fake_out = "NAME AUTOCONNECT\nMy Wifi          yes\nGuest no\n"
        with patch.object(inst, "run", return_value=subprocess.CompletedProcess(args="", returncode=0, stdout=fake_out, stderr="")):
            inst.get_auto_connect_status()
        self.assertEqual("yes", inst.wifi_connections["My Wifi"]["autoconnect"])
        self.assertEqual("no", inst.wifi_connections["Guest"]["autoconnect"])

    def test_add_connection_infos_parses_nmconnection(self):
        inst = self.make_instance()
        content = "id=myssid\nssid=myssid\npsk=mypassword\n"
        with tempfile.NamedTemporaryFile("w+", delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name
        try:
            inst.add_connection_infos(tmp_path)
        finally:
            os.unlink(tmp_path)
        self.assertEqual("mypassword", inst.wifi_connections["myssid"]["password"])

    def test_list_wifi_connection_names_filters_wireless(self):
        inst = self.make_instance()
        fake_out = "home:802-11-wireless\nvpn:tun\nlab:802-11-wireless\n"
        with patch.object(inst, "run", return_value=subprocess.CompletedProcess(args="", returncode=0, stdout=fake_out, stderr="")):
            names = inst.list_wifi_connection_names()
        self.assertListEqual(["home", "lab"], names)

    def test_purge_wifi_connections_deletes_and_clears_cache(self):
        inst = self.make_instance()
        inst.wifi_connections = {"old": {"password": "x"}}
        with patch.object(inst, "list_wifi_connection_names", return_value=["old", "guest"]) as lister, \
             patch.object(inst, "delete_wifi_connection") as deleter:
            inst.purge_wifi_connections()
        deleter.assert_any_call("old")
        deleter.assert_any_call("guest")
        self.assertEqual({}, inst.wifi_connections)

    def test_configuration_callback_applies_after_three_keys(self):
        inst = self.make_instance()
        applied = {}

        def fake_purge():
            applied["purged"] = True

        def fake_add(ssid, pwd, auto):
            applied["ssid"] = ssid
            applied["pwd"] = pwd
            applied["auto"] = auto

        inst.purge_wifi_connections = fake_purge
        inst.add_connection_to_nmcli = fake_add

        Setting = types.SimpleNamespace
        inst.configuration_callback(Setting(key="wifiSSID", value="net1"))
        inst.configuration_callback(Setting(key="wifiPassword", value="secret123"))
        inst.configuration_callback(Setting(key="wifiTransferEnabled", value="true"))

        self.assertTrue(applied.get("purged"))
        self.assertEqual("net1", applied.get("ssid"))
        self.assertEqual("secret123", applied.get("pwd"))
        self.assertEqual("yes", applied.get("auto"))
        self.assertEqual(0, inst.config_callback_counter)

    def test_get_wifi_config_skips_when_ssid_empty(self):
        inst = self.make_instance()
        inst.get_config_value = MagicMock(side_effect=["", "", "false"])
        inst.purge_wifi_connections = MagicMock()
        inst.add_connection_to_nmcli = MagicMock()

        inst.get_wifi_config()

        inst.purge_wifi_connections.assert_not_called()
        inst.add_connection_to_nmcli.assert_not_called()


if __name__ == "__main__":
    unittest.main()
