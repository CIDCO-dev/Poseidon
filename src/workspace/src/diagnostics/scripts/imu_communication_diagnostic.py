# imu_communication_diagnostic.py
#!/usr/bin/env python3
import math
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Imu
from diagnostics_test_base import DiagnosticsTest


class ImuCommunicationDiagnostic(DiagnosticsTest):
    """
    Checks the reception of IMU messages on a topic (default: '/imu/data').
    - OK if at least (freq * window * expected_ratio) messages are received
    - WARN if > 0 but below the threshold
    - ERROR if no message is received during the observation window
    """

    def __init__(
        self,
        name="IMU Communication",
        message_frequency=100,         # expected rate (Hz)
        topic="/imu/data",
        timeout_s=1.0,                 # observation window
        expected_ratio=0.8             # tolerance (80% of theoretical)
    ):
        super().__init__()
        self._name = name
        self.topic = topic
        self.message_frequency = int(message_frequency)
        self.timeout_s = float(timeout_s)
        self.expected_ratio = float(expected_ratio)

        self._count = 0
        self._sub = None

    def _cb(self, msg: Imu):
        self._count += 1

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = f"topic://{self.topic}"

        self._count = 0
        self._sub = rospy.Subscriber(self.topic, Imu, self._cb, queue_size=10)

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
            status.message = f"No IMU message received in {elapsed:.2f} seconds"
        elif self._count < max(1, int(math.floor(self.message_frequency * self.timeout_s * self.expected_ratio))):
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._count} msg(s) in {elapsed:.2f}s (below expected rate)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"{self._count} msg(s) received in {elapsed:.2f} seconds"

        return status
