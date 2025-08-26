#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus

class DiagnosticsTest:
    def update(self):
        # À surcharger par les sous-classes
        return DiagnosticStatus(level=DiagnosticStatus.OK, name="BaseTest", message="OK")

