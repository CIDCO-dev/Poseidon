# gnss_communication_diagnostic.py
#!/usr/bin/env python3
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import NavSatFix
from diagnostics_test_base import DiagnosticsTest


class GnssCommunicationDiagnostic(DiagnosticsTest):

    """
    Checks the reception of NavSatFix messages on a topic (default: 'fix').
    - OK if at least (message_frequency * timeout_s) messages are received
    - WARN if > 0 but below the threshold
    - ERROR if no message is received during the observation window
    """


    def __init__(self, name="GNSS Communication", message_frequency=1, topic="fix", timeout_s=1.5):
        super().__init__()
        self._name = name
        self.topic = topic
        self.message_frequency = int(message_frequency)
        self.timeout_s = float(timeout_s)

        self._count = 0
        self._sub = None

    def _cb(self, msg: NavSatFix):
        self._count += 1

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = f"topic://{self.topic}"

        self._count = 0
        self._sub = rospy.Subscriber(self.topic, NavSatFix, self._cb, queue_size=10)

        start = rospy.Time.now()
        rate = rospy.Rate(20.0)  # 50 ms
        try:
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start).to_sec()

                
                expected = max(1, int(round(self.message_frequency * self.timeout_s)))
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
        elif self._count < max(1, int(round(self.message_frequency * self.timeout_s))):
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._count} msg(s) in {elapsed:.2f}s (below expected rate)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"{self._count} msg(s) received in {elapsed:.2f} seconds"

        return status
