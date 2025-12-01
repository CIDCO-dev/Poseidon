#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from binary_stream_msg.msg import Stream  
from diagnostics_test_base import DiagnosticsTest


class BinaryStreamGnssDiagnostic(DiagnosticsTest):
    def __init__(self, name="BinaryStreamGnss", message_frequency=10):
        self.name = name
        self.message_frequency = message_frequency
        self.message_count = 0
        self.sleep_duration = 1.5
        self.subscriber = None

    def callback(self, msg):
        if len(msg.stream) == msg.vector_length:
            self.message_count += 1

    def update(self):
        try:
            self.message_count = 0
            self.subscriber = rospy.Subscriber("gnss_bin_stream", Stream, self.callback)

            start_time = rospy.Time.now()
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                elapsed_time = (rospy.Time.now() - start_time).to_sec()

                if self.message_count >= (self.message_frequency * self.sleep_duration):
                    break

                if elapsed_time >= self.sleep_duration:
                    break

                rate.sleep()

            self.subscriber.unregister()

            if self.message_count > 0:
                return DiagnosticStatus(
                    level=DiagnosticStatus.OK,
                    name=self.name,
                    message=f"{self.message_count} messages reçus en {elapsed_time:.2f} secondes"
                )
            else:
                return DiagnosticStatus(
                    level=DiagnosticStatus.ERROR,
                    name=self.name,
                    message=f"Aucun message reçu en {elapsed_time:.2f} secondes"
                )

        except Exception as ex:
            rospy.logerr(str(ex))
            if self.subscriber:
                self.subscriber.unregister()
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                name=self.name,
                message=f"Exception: {str(ex)}"
            )
