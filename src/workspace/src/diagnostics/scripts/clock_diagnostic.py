# clock_diagnostic.py
#!/usr/bin/env python3
import time
import rospy
from datetime import datetime
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest

# Le module "gps" provient de python3-gps (gpsd)
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE

class ClockDiagnostic(DiagnosticsTest):
    """
    Vérifie la latence entre l'heure système et l'heure GPS fournie par gpsd.
    - OK si |system - gps| <= 30 ms
    - WARN sinon
    - ERROR si gpsd absent ou pas de TPV/time en 2s
    """
    def __init__(self, name="Clock Diagnostic", threshold_ms=30, timeout_s=2.0):
        super().__init__()
        self._name = name
        self.threshold_ms = int(threshold_ms)
        self.timeout_s = float(timeout_s)

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = "gpsd@localhost:2947"

        try:
            session = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No GPSD running ({e})"
            return status

        start = time.time()
        latency_ms = None

        try:
            while (time.time() - start) < self.timeout_s and not rospy.is_shutdown():
                report = session.next()  # peut lever StopIteration
                # On cherche un rapport TPV avec un champ "time"
                if isinstance(report, dict):
                    cls = report.get("class")
                    ts_str = report.get("time")
                else:
                    # Certains bindings exposent en attributs
                    cls = getattr(report, "class", None)
                    ts_str = getattr(report, "time", None)

                if cls == "TPV" and ts_str:
                    # gpsd donne une ISO8601, souvent "...Z"
                    if ts_str.endswith("Z"):
                        ts_str = ts_str[:-1]
                    try:
                        gps_epoch = datetime.fromisoformat(ts_str).timestamp()
                    except ValueError:
                        # Format fallback: "YYYY-mm-ddTHH:MM:SS" sans microsecondes
                        gps_epoch = datetime.strptime(ts_str.split(".")[0], "%Y-%m-%dT%H:%M:%S").timestamp()

                    sys_epoch = time.time()
                    latency_ms = abs(int(round((sys_epoch - gps_epoch) * 1000.0)))
                    break
        except StopIteration:
            pass
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"gpsd error: {e}"
            return status

        if latency_ms is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "No TPV/time from gpsd within 2s"
            return status

        status.message = f"latency = {latency_ms} milliseconds"
        status.level = DiagnosticStatus.OK if latency_ms <= self.threshold_ms else DiagnosticStatus.WARN
        return status
