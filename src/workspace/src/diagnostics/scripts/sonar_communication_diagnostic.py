# sonar_communication_diagnostic.py
#!/usr/bin/env python3
import math
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PointStamped
from diagnostics_test_base import DiagnosticsTest


class SonarCommunicationDiagnostic(DiagnosticsTest):
    """
    Vérifie la réception de messages sondeurs sur un topic (par défaut '/depth').
    - OK si on reçoit >= (freq * fenêtre * expected_ratio)
    - WARN si > 0 mais en dessous du seuil
    - ERROR si 0 message pendant la fenêtre
    """

    def __init__(
        self,
        name="Sonar Communication",
        message_frequency=10,      # expected rate (Hz)
        topic="/depth",
        timeout_s=1.0,             # observation window
        expected_ratio=0.8         # tolerance (80% of theoretical)
    ):
        super().__init__()
        self._name = name
        self.topic = topic
        self.message_frequency = int(message_frequency)
        self.timeout_s = float(timeout_s)
        self.expected_ratio = float(expected_ratio)

        self._count = 0
        self._sub = None

    def _cb(self, msg: PointStamped):
        self._count += 1

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = f"topic://{self.topic}"

        # reset window
        self._count = 0
        self._sub = rospy.Subscriber(self.topic, PointStamped, self._cb, queue_size=10)

        start = rospy.Time.now()
        rate = rospy.Rate(50.0)  # 20 ms
        try:
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start).to_sec()

                expected_raw = self.message_frequency * self.timeout_s * self.expected_ratio
                expected = max(1, int(math.floor(expected_raw)))

                if self._count >= expected:
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

        if self._count == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No message received in {elapsed:.2f} seconds"
        elif self._count < max(1, int(math.floor(self.message_frequency * self.timeout_s * self.expected_ratio))):
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._count} msg(s) in {elapsed:.2f}s (below expected rate)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"{self._count} msg(s) received in {elapsed:.2f} seconds"

        return status
