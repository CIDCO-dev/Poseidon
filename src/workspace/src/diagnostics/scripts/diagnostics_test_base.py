#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus

class DiagnosticsTest:
    def update(self):
        # To be overridden by subclasses
        return DiagnosticStatus(level=DiagnosticStatus.OK, name="BaseTest", message="OK")

