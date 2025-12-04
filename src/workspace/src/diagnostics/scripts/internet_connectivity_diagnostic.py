#!/usr/bin/env python3

import requests

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest


class InternetConnectivityDiagnostic(DiagnosticsTest):
    def __init__(self, name="Internet Connectivity", url="http://example.com", timeout_s=3.0):
        super().__init__()
        self.name = name
        self.url = url
        self.timeout_s = timeout_s

    def update(self):
        status = DiagnosticStatus()
        status.name = self.name

        try:
            resp = requests.get(self.url, timeout=self.timeout_s)
            if resp.status_code == 200:
                status.level = DiagnosticStatus.OK
                status.message = f"Reachable: {self.url} (status {resp.status_code})"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = f"HTTP {resp.status_code} from {self.url}"
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Failed to reach {self.url}: {e}"

        return status
