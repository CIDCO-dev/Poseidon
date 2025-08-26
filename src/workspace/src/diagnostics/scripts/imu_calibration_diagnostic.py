# imu_calibration_diagnostic.py
#!/usr/bin/env python3
import math
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Imu
from diagnostics_test_base import DiagnosticsTest

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_multiply


def q_to_list(q):
    return [q.x, q.y, q.z, q.w]


class ImuCalibrationDiagnostic(DiagnosticsTest):
    """
    OK if |pitch| <= threshold_deg and |roll| <= threshold_deg during the observation window.
    Otherwise ERROR, with R/P/H reported (in degrees).
    """

    def __init__(
        self,
        name="IMU Calibrated",
        message_frequency=100,           # Hz 
        topic="imu/data",
        sleep_duration=1.0,              # s
        threshold_deg=1.5,               # in degres 
        target_frame="base_link",
        source_frame="imu",
    ):
        super().__init__()
        self._name = name
        self.topic = topic
        self.message_frequency = int(message_frequency)
        self.sleep_duration = float(sleep_duration)
        self.threshold_deg = float(threshold_deg)
        self.target_frame = target_frame
        self.source_frame = source_frame

        self._sub = None
        self._count = 0
        self._calibrated = True
        self._r_deg = 0.0
        self._p_deg = 0.0
        self._h_deg = 0.0

        # TF2
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def _cb(self, imu: Imu):
        self._count += 1
        try:
            # TF (base_link <- imu), time 0 = last
            t = self._tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rospy.Time(0), timeout=rospy.Duration(0.1)
            )
            q_tf = q_to_list(t.transform.rotation)
            q_imu = q_to_list(imu.orientation)

            # Orientation in base_link = q_tf * q_imu
            q_base = quaternion_multiply(q_tf, q_imu)

            # tf returns (roll, pitch, yaw) in radians
            r, p, h = euler_from_quaternion(q_base)
            self._r_deg = math.degrees(r)
            self._p_deg = math.degrees(p)
            self._h_deg = math.degrees(h)

            if abs(self._p_deg) > self.threshold_deg or abs(self._r_deg) > self.threshold_deg:
                self._calibrated = False

        except Exception as e:
            rospy.logdebug(f"IMU calib TF lookup failed: {e}")

    def update(self):
        status = DiagnosticStatus()
        status.name = self._name
        status.hardware_id = f"tf:{self.source_frame}->{self.target_frame} topic://{self.topic}"

        # (re)init window
        self._count = 0
        self._calibrated = True
        self._r_deg = self._p_deg = self._h_deg = 0.0

        self._sub = rospy.Subscriber(self.topic, Imu, self._cb, queue_size=10)

        start = rospy.Time.now()
        expected = max(1, int(round(self.message_frequency * self.sleep_duration)))
        rate = rospy.Rate(20.0)  # 50 ms

        try:
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start).to_sec()
                if self._count >= expected:
                    break
                if elapsed >= self.sleep_duration:
                    break
                rate.sleep()
        finally:
            try:
                if self._sub is not None:
                    self._sub.unregister()
            except Exception:
                pass

        if self._count == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No IMU message received in {self.sleep_duration:.2f}s"
            return status

        if self._calibrated:
            status.level = DiagnosticStatus.OK
            status.message = "Calibrated"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                f"Not calibrated.\n"
                f"Roll: {self._r_deg:.2f}\n"
                f"Pitch: {self._p_deg:.2f}\n"
                f"Heading: {self._h_deg:.2f}"
            )
        return status
