#!/usr/bin/env python3
import rospy
import asyncio
import websockets
import json
import rosgraph.masterapi
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostics_test_base import DiagnosticsTest
from api_connection_diagnostic import ApiConnectionDiagnostic
from binary_stream_gnss_diagnostic import BinaryStreamGnssDiagnostic
from clock_diagnostic import ClockDiagnostic
from gnss_communication_diagnostic import GnssCommunicationDiagnostic
from gnss_fix_diagnostic import GnssFixDiagnostic
from imu_calibration_diagnostic import ImuCalibrationDiagnostic
from imu_communication_diagnostic import ImuCommunicationDiagnostic
from serial_number_diagnostic import SerialNumberDiagnostic
from sonar_communication_diagnostic import SonarCommunicationDiagnostic



class DiagnosticsServer:
    def __init__(self):
        self.tests = []
        self.loop = None
        self.server = None

    def add_test(self, test):
        self.tests.append(test)


    async def websocket_handler(self, websocket, path):
        rospy.loginfo("Client WebSocket connecté")

        async for message in websocket:
            rospy.loginfo(f"Message reçu: {message}")
            try:
                req = json.loads(message)
                command = req.get("command")

                if command == "updateDiagnostic":
                    msg = DiagnosticArray()
                    for test in self.tests:
                        msg.status.append(test.update())

                    diagnostics = []
                    for s in msg.status:
                        diagnostics.append({
                            "status": s.level == DiagnosticStatus.OK,
                            "name": s.name,
                            "message": s.message
                        })

                    await websocket.send(json.dumps({"diagnostics": diagnostics}))

                elif command == "getRunningNodes":
                    master = rosgraph.masterapi.Master('/diagnostics_websocket')
                    nodes = master.getSystemState()
                    running_nodes = set()

                    for _, publishers in nodes:
                        for pub in publishers:
                            running_nodes.add(pub)

                    await websocket.send(json.dumps({"running_nodes": list(running_nodes)}))

                else:
                    rospy.logwarn(f"Commande inconnue: {command}")

            except Exception as e:
                rospy.logerr(f"Erreur WebSocket : {e}")



    def start_server(self, port=9099):
        """Lauch WebSocket server in asyncio loop."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.server = websockets.serve(self.websocket_handler, "0.0.0.0", port)
        self.loop.run_until_complete(self.server)
        rospy.loginfo(f"DiagnosticsServer WebSocket listen to port {port}")
        self.loop.run_forever()

    def stop_server(self):
        """Stop WebSocket server."""
        if self.loop and self.loop.is_running():
            rospy.loginfo("Stopping WebSocket server...")
            self.loop.call_soon_threadsafe(self.loop.stop)

    def run_in_thread(self, port=9099):
        """Starts the server in a separate thread so it doesn't block ROS."""
        t = threading.Thread(target=self.start_server, args=(port,))
        t.daemon = True
        t.start()


def main():
    rospy.init_node('diagnostics')
    server = DiagnosticsServer()

    server.add_test(ApiConnectionDiagnostic())
    server.add_test(BinaryStreamGnssDiagnostic(message_frequency=10))
    server.add_test(ClockDiagnostic())
    server.add_test(GnssCommunicationDiagnostic(name="GNSS Communication", message_frequency=2, topic="fix", timeout_s=2))
    server.add_test(GnssFixDiagnostic(name="GNSS Fix", message_frequency=2, topic="fix", timeout_s=2, min_fix_ratio=0.90))
    server.add_test(ImuCalibrationDiagnostic(
        name="IMU Calibrated",
        message_frequency=100,   
        topic="imu/data",
        sleep_duration=1.0,
        threshold_deg=1.5,
        target_frame="base_link",
        source_frame="imu",
    ))
    server.add_test(ImuCommunicationDiagnostic(
        name="IMU Communication",
        message_frequency=100,  
        topic="/imu/data",
        timeout_s=1.0,
        expected_ratio=0.8
    ))
    server.add_test(SerialNumberDiagnostic(name="Serial Number Pattern Validation"))
    server.add_test(SonarCommunicationDiagnostic(
        name="Sonar Communication",
        message_frequency=1,   # car /depth ≈ 1 Hz
        topic="/depth",
        timeout_s=1.5,
        expected_ratio=0.8
    ))


    # Start the WebSocket server in a separate thread
    server.run_in_thread(port=9099)

    # Cleanly stop the server when ROS shuts down
    rospy.on_shutdown(server.stop_server)

    rospy.loginfo("DiagnosticsServer WebSocket started with full ROS integration")
    rospy.spin()


if __name__ == '__main__':
    main()
