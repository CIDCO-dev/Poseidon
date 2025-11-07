#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest
from setting_msg.srv import ConfigurationService
import socket
import requests
import rospy


def trim_spaces(s):
    return s.strip()


class ApiConnectionDiagnostic(DiagnosticsTest):
    def __init__(self):
        super().__init__()
        self.name = "API Connection"
        self.host = ""
        self.port = "8080"
        self.node = rospy
        self.client = rospy.ServiceProxy('get_configuration', ConfigurationService)

    def get_config(self):
        try:
            rospy.wait_for_service('get_configuration', timeout=3.0)

            # Get host
            try:
                resp = self.client("apiServer")
                self.host = trim_spaces(resp.value)
            except Exception as e:
                rospy.logerr(f"Error retrieving apiServer: {e}")
                self.host = ""

            # Get port
            try:
                resp = self.client("apiPort")
                self.port = trim_spaces(resp.value)
            except Exception as e:
                rospy.logerr(f"Error retrieving apiPort: {e}")
                self.port = "8080"

        except rospy.ROSException:
            rospy.logwarn("Service get_configuration not available")
            self.host = ""
            self.port = "8080"



    

    def update(self):
        status = DiagnosticStatus()
        status.name = self.name

        self.get_config()

        if not self.host:
            status.level = DiagnosticStatus.ERROR
            status.message = "API server not defined"
            return status

        url = f"http://{self.host}:{self.port}/"

        try:
            r = requests.get(url, timeout=2)
            if r.status_code == 200:
                status.level = DiagnosticStatus.OK
                status.message = f"API reachable at {url}"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = f"API responded with status {r.status_code}"
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Failed to reach API at {url}: {e}"

        return status
