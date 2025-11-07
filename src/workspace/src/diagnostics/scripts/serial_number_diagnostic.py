# serial_number_diagnostic.py
#!/usr/bin/env python3
import re
import socket
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest


class SerialNumberDiagnostic(DiagnosticsTest):
    """
    Checks that the hostname matches the pattern: ^[A-Za-z]{2,8}-\d{6}-\d{3}$
    Valid examples: AB-123456-001, DATALOG-240101-123
    """
    
    def __init__(self, name="Serial Number Pattern Validation", pattern=r"^[A-Za-z]{2,8}-\d{6}-\d{3}$"):
        super().__init__()
        self._name = name
        self._pattern = re.compile(pattern)

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = "hostname"

        try:
            host = socket.gethostname() or ""
            # If FQDN, keep only the first label (before the first '.')
            host_short = host.split(".")[0]

            if not host_short:
                status.level = DiagnosticStatus.ERROR
                status.message = "Could not get hostname — no serial number set"
                return status

            if self._pattern.match(host_short):
                status.level = DiagnosticStatus.OK
                status.message = f"Valid serial number pattern: {host_short}"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = f"Not valid serial number pattern: {host_short}"

            return status

        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Exception while checking hostname: {e}"
            return status
