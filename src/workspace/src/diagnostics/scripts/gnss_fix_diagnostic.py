# gnss_fix_diagnostic.py
#!/usr/bin/env python3
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import NavSatFix
from diagnostics_test_base import DiagnosticsTest


class GnssFixDiagnostic(DiagnosticsTest):
    """
    Vérifie qu'une proportion suffisante des NavSatFix ont un 'fix'.
    - Critère par défaut : msg.status.status >= 0  (FIX/SBAS/GBAS)
      (Tu peux forcer l'ancien comportement C++ avec use_service_field=True)
    - OK si ratio_fix >= min_fix_ratio (ex: 0.90)
    - ERROR sinon, ou si aucun message reçu dans la fenêtre d'observation
    """

    def __init__(
        self,
        name="GNSS Fix",
        message_frequency=1,   # Hz attendu
        topic="fix",
        timeout_s=1.5,
        min_fix_ratio=0.90,
        use_service_field=False
    ):
        super().__init__()
        self._name = name
        self.topic = topic
        self.message_frequency = int(message_frequency)
        self.timeout_s = float(timeout_s)
        self.min_fix_ratio = float(min_fix_ratio)
        self.use_service_field = bool(use_service_field)

        self._total = 0
        self._fix = 0
        self._sub = None

    def _cb(self, msg: NavSatFix):
        self._total += 1
        try:
            if self.use_service_field:
                # Comportement du C++ fourni (probablement une confusion)
                ok = (msg.status.service >= 0)
            else:
                # Logique correcte : STATUS_NO_FIX = -1, FIX/SBAS/GBAS >= 0
                ok = (msg.status.status >= 0)
            if ok:
                self._fix += 1
        except Exception:
            pass

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = f"topic://{self.topic}"

        self._total = 0
        self._fix = 0
        self._sub = rospy.Subscriber(self.topic, NavSatFix, self._cb, queue_size=10)

        start = rospy.Time.now()
        rate = rospy.Rate(20.0)  # 50 ms
        try:
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start).to_sec()

                # minimum number of messages to receive for a proper evaluation
                expected = max(1, int(round(self.message_frequency * self.timeout_s)))
                if self._total >= expected:
                    break

                if elapsed >= self.timeout_s:
                    break

                rate.sleep()
        finally:
            try:
                if self._sub is not None:
                    self._sub.unregister()
            except Exception:
                pass

        elapsed = (rospy.Time.now() - start).to_sec()

        if self._total == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No message received in {elapsed:.2f} seconds"
            return status

        ratio = self._fix / float(self._total)
        ratio_pct = ratio * 100.0

        if ratio >= self.min_fix_ratio:
            status.level = DiagnosticStatus.OK
            status.message = f"{ratio_pct:.1f}% of messages have GNSS fix ({self._fix}/{self._total})"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No GNSS fix (only {ratio_pct:.1f}% with fix; {self._fix}/{self._total})"

        return status
