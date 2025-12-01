#!/usr/bin/env python3

import socket

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest


class DnsResolutionDiagnostic(DiagnosticsTest):
    def __init__(self, name="DNS Resolution", hostname="google.com"):
        super().__init__()
        self.name = name
        self.hostname = hostname

    def update(self):
        status = DiagnosticStatus()
        status.name = self.name

        try:
            socket.gethostbyname(self.hostname)
            status.level = DiagnosticStatus.OK
            status.message = f"Resolved {self.hostname}"
        except socket.gaierror as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"DNS resolution failed for {self.hostname}: {e}"
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"DNS resolution error for {self.hostname}: {e}"

        return status
