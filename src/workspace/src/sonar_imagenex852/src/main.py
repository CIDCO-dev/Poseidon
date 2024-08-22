#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import PointStamped
from setting_msg.srv import ConfigurationService
from setting_msg.msg import Setting

# Import necessary messages and services if needed

class SonarNode:
    def __init__(self):
        self.sonar_topic = rospy.Publisher("depth", PointStamped, queue_size=1000)
        self.sonar_topic_enu = rospy.Publisher("depth_enu", PointStamped, queue_size=1000)
        self.configuration_client = rospy.ServiceProxy("get_configuration", ConfigurationService)
        self.sequence_number = 0
        self.sonar_start_gain = [0]  
        self.sonar_range = [0]  
        self.sonar_absorption = [0]  
        self.sonar_pulse_length = [0]  
        # Initialize the node
        rospy.init_node('sonar_node')
        self.last_data_time = rospy.Time.now()
        self.configuration_changed = {
            "sonarStartGain": False,
            "sonarRange": False,
            "sonarAbsorption": False,
            "sonarPulseLength": False
        }
        self.data_received = False
        
        # Initialize the serial port
        try:
            self.serial_port = serial.Serial('/dev/sonar', 115200)
        except:
            rospy.logerr(f"Sonar : unable to open port{self.serial_port}")
            exit

        self.send_cmd_data()

        # Create an array to store received data
        self.received_data = []
        
        rospy.Subscriber("configuration", Setting, self.configuration_change)

    def configuration_change(self, setting):
        key = setting.key
        try:
            value = int(setting.value)
        except:
            value = 0
        if key == "sonarStartGain" and self.sonar_start_gain[0] != value:
            self.sonar_start_gain[0] = value
            self.configuration_changed["sonarStartGain"] = True
        elif key == "sonarRange" and self.sonar_range[0] != value:
            self.sonar_range[0] = value
            self.configuration_changed["sonarRange"] = True
        elif key == "sonarAbsorbtion" and self.sonar_absorption[0] != value:
            self.sonar_absorption[0] = value
            self.configuration_changed["sonarAbsorption"] = True
        elif key == "sonarPulseLength" and self.sonar_pulse_length[0] != value:
            self.sonar_pulse_length[0] = value
            self.configuration_changed["sonarPulseLength"] = True

        print(f"Configuration change - Key: {key}, Value: {value}")  
        if all(self.configuration_changed.values()) and not self.data_received:
            self.send_cmd_data()
            self.configuration_changed = {key: False for key in self.configuration_changed}

    def arrondir_sonar_range(self, sonarRange):
        if sonarRange in [5, 10, 20, 30, 40, 50]:
            return sonarRange

        if sonarRange > 50:
            return 50

        nearest_values = [5, 10, 20, 30, 40, 50]
        nearest_value = min(nearest_values, key=lambda x: abs(x - sonarRange))
        
        return nearest_value
        
    def send_cmd_data(self):
        sonarRange = self.sonar_start_gain[0]
        sonarStartGain = self.sonar_range[0] 
        sonarAbsorbtion = self.sonar_absorption[0]
        sonarPulseLength = self.sonar_pulse_length[0]
        if sonarRange == 0:
            sonarRange = 50
        if sonarStartGain >= 41:
            sonarStartGain = 40
        if sonarAbsorbtion != 20:
            sonarAbsorbtion = 20
        if sonarPulseLength >= 256:
            sonarPulseLength = 255
        if sonarPulseLength == 0:
            sonarPulseLength = 1        
        self.arrondir_sonar_range(sonarRange)
        cmd_data = bytes([0xFE, 0x44, 0x11, sonarRange, 0x00, 0x00, 0x00, 0x00, sonarStartGain, 0x00, 20, 0x00, 0x00, 0x00, sonarPulseLength, 0, 0x00, 0x00, 0x07, 0, 0, 0, 1, 0, 0, 0, 0xFD])
        #cmd_data = bytes([0xFE, 0x44, 0x11, 5,          0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 20,              0x00, 0x00, 0x00, 150,              2, 0x00, 0x00, 0x07, 0, 0, 0, 1, 0, 0, 0, 0xFD])
        self.serial_port.write(cmd_data) 
        for byte in cmd_data:
            print('x{:02x}'.format(byte), end=' ')        

    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            data = self.serial_port.read(1)  # Read one byte at a time
            if data == b'\xFC':
                self.process_data()  # Call the processing function
                self.last_data_time = rospy.Time.now() 
            else:
                self.received_data.append(data)
            # Check if more than 1 second has passed since the last data
            current_time = rospy.Time.now()
            delta_time = float((current_time - self.last_data_time).to_sec())
            if all(self.configuration_changed.values()):
                self.send_cmd_data()
            
            if delta_time > 1.1:
                self.received_data = []
                self.process_data()

    def update_depth(self, depth):
        msg = PointStamped()
        msg.header.seq = self.sequence_number
        msg.header.frame_id = "sonar"
        current_time = rospy.Time.now()
        msg.header.stamp = current_time
        msg.point.z = depth
        self.sonar_topic.publish(msg)

        # Also publish in ENU frame
        msg.header.frame_id = "sonar_enu"
        msg.point.z = -msg.point.z
        self.sonar_topic_enu.publish(msg)
        print(f"Depth: {depth} at {current_time.secs}.{current_time.nsecs}")


    def process_data(self):
        #received_message = b''.join(self.received_data)
        #print("Received message:", received_message)
        #print(len(self.received_data))
        # Check if the length of received_data is at least 12 (minimum required length)
        if len(self.received_data) >= 12 and self.received_data[1] == b'P':
                
            # Extract and calculate the range and data bytes
            byte4 = self.received_data[4][0]
            byte7 = self.received_data[7][0]
            byte9 = self.received_data[9][0]
            byte8 = self.received_data[8][0]
            byte11 = self.received_data[11][0]
            byte10 = self.received_data[10][0]

            # value for automatic mode
            automatic_mode = (byte4 & 0x04) >> 2
            # value for sonar detected
            sonar_present = byte4 & 0x01

            # Calculate Prof Rng High Byte and Prof Rng Low Byte
            prof_rng_high_byte = ((byte9 & 0x7E) >> 1)
            prof_rng_low_byte = (((byte9 & 0x01) << 7) | (byte8 & 0x7F))
            profile_range = (prof_rng_high_byte << 8) | prof_rng_low_byte

            # Calculate Data Bytes High Byte and Data Bytes Low Byte
            data_bytes_high_byte = ((byte11 & 0x7E) >> 1)
            data_bytes_low_byte = (((byte11 & 0x01) << 7) | (byte10 & 0x7F))
            data_bytes = (data_bytes_high_byte << 8) | data_bytes_low_byte

            depth = float(profile_range) / 100
            self.data_received = True
            if automatic_mode == 1 and sonar_present == 1:
                if not any(self.configuration_changed.values()) and self.data_received:
                    self.update_depth(depth)
                        #self.send_cmd_data()
                        #self.configuration_changed = {key: False for key in self.configuration_changed}
                    self.data_received = False

                
            elif automatic_mode == 0:
                rospy.logerr("Automatic trigger mode not supported.")
            elif sonar_present == 0:
                rospy.logerr("Echosounder not detected")
        else:
            self.update_depth(-1.0)
            
        self.received_data = []

if __name__ == '__main__':
    try:
        node = SonarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
