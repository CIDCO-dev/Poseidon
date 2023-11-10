import rospy
from geometry_msgs.msg import PointStamped
from setting_msg.srv import ConfigurationService
from setting_msg.msg import Setting
from std_msgs.msg import Header
import serial
import struct
from threading import Lock, Thread
from time import sleep

class Imagenex852:
    def __init__(self, device_path):
        self.device_path = device_path
        self.sonar_topic = rospy.Publisher("depth", PointStamped, queue_size=1000)
        self.sonar_topic_enu = rospy.Publisher("depth_enu", PointStamped, queue_size=1000)
        self.configuration_client = rospy.ServiceProxy("get_configuration", ConfigurationService)
        rospy.loginfo("Fetching sonar configuration...")
        self.get_configuration()
        self.device_file = None
        self.mtx = Lock()
        self.sequence_number = 0
        self.delay_nanoseconds = 0  # FIXME: get from a ROS parameter
        self.initial_command_sent = False  # Flag to track if the initial command has been sent
        self.last_data_received_time = rospy.Time.now()

    class Imagenex852SwitchDataCommand:
        def __init__(self):
            self.magic = [0, 0]
            self.head_id = 0
            self.range = 0
            self.reserved = [0, 0]
            self.master_slave = 0
            self.reserved2 = 0
            self.start_gain = 0
            self.reserved3 = 0
            self.absorption = 0
            self.reserved4 = [0, 0, 0]
            self.pulse_length = 0
            self.profile_minimum_range = 0
            self.reserved5 = [0, 0]
            self.trigger_control = 0
            self.data_points = 0
            self.reserved6 = [0, 0]
            self.profile = 0
            self.reserved7 = 0
            self.switch_delay = 0
            self.frequency = 0
            self.termination_byte = 0
        
        
    def __del__(self):
        if self.device_file:
            self.device_file.close()

    def get_configuration(self):
        config_keys = ["sonarStartGain", "sonarRange", "sonarAbsorbtion", "sonarPulseLength"]
        value_ptrs = [self.sonar_start_gain, self.sonar_range, self.sonar_absorption, self.sonar_pulse_length]

        for i in range(4):
            value_string = self.get_config_value(config_keys[i])
            self.set_config_value(value_string, value_ptrs[i])

    def get_config_value(self, key):
        try:
            response = self.configuration_client(key)
            return response.value
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return ""

    def set_config_value(self, value_str, val):
        with self.mtx:
            val[0] = int(value_str)

    def configuration_change(self, setting):
        key = setting.key
        value = int(setting.value)
        run..initial_command_sent = False
        with self.mtx:
            if key == "sonarStartGain":
                self.sonar_start_gain[0] = value
            elif key == "sonarRange":
                self.sonar_range[0] = value
            elif key == "sonarAbsorbtion":
                self.sonar_absorption[0] = value
            elif key == "sonarPulseLength":
                self.sonar_pulse_length[0] = value

    def process_messages(self):
        rospy.Subscriber("configuration", Setting, self.configuration_change)
        #rate = rospy.Rate(1.5)
        while not rospy.is_shutdown():
            # Check if a configuration message has been received
            #if self.config_received.wait(timeout=0.1):
                # Handle the configuration change
                #self.config_received.clear()
            #rate.sleep()
            rospy.spin_once()

    def send_sonar_command(self):
        # Send the initial command to the sonar
        cmd = Imagenex852SwitchDataCommand()
        cmd.magic[0] = 0xFE
        cmd.magic[1] = 0x44
        cmd.headId = 0x11
        cmd.masterSlave = 0x43
    
        # Set command parameters
        with self.mtx:
            cmd.range = self.sonar_range[0]
            cmd.startGain = self.sonar_start_gain[0]
            cmd.absorption = self.sonar_absorption[0]
            cmd.pulseLength = self.sonar_pulse_length[0]
        
        # Write the command to the serial port
        nb_bytes = 0
    
        if self.device_file:
            try:
                self.device_file.write(bytearray(struct.pack('27B', *cmd)))
                self.device_file.flush()
                nb_bytes = 27
            except serial.SerialException as e:
                rospy.logerr(f"Failed to write switch data command: {e}")
        else:
            rospy.logerr("Device file is not open. Cannot send the command.")
    
        if nb_bytes == 27:
            # Command sent successfully
            rospy.loginfo("Initial command sent successfully")
        else:
            rospy.logerr(f"Cannot write switch data command ({nb_bytes} bytes written)")
            raise Exception("Failed to send the command")
 

        # if ((nbBytes = write(deviceFile, &cmd, sizeof(Imagenex852SwitchDataCommand))) == 27) {
            # // Command sent successfully
        # }
        # else{
            # ROS_ERROR("Cannot write switch data command (%d bytes written)", nbBytes);
            # throw std::system_error();
        # }
    # }

    def run(self):
        try:
            self.device_file = serial.Serial(self.device_path, baudrate=115200, timeout=1)
            rospy.loginfo(f"Sonar file opened on {self.device_path}")

            # Send initial command if not already sent
            if not self.initial_command_sent:
                self.send_sonar_command()
                self.initial_command_sent = True

            # Launch message pump
            message_thread = Thread(target=self.process_messages)
            message_thread.start()

            error_rate = rospy.Rate(1)

            rospy.loginfo(f"Sonar node ready to process data")
            while not rospy.is_shutdown():
                # record ping start
                #ping_start = rospy.Time.now()

                msg = PointStamped()
                msg.header.seq = self.sequence_number
                msg.header.frame_id = "sonar"
                current_time = rospy.Time.now()
                msg.header.stamp = rospy.Time.now()
                rospy.loginfo(f"Ping: {current_time.secs}.{current_time.nsecs}")
                print("ping")
                try:
                    # Publish depth point measurement
                    msg.point.z = self.measure_depth(msg, 0)
                    self.sonar_topic.publish(msg)

                    # Also publish in ENU frame
                    msg.header.frame_id = "sonar_enu"
                    msg.point.z = -msg.point.z
                    self.sonar_topic_enu.publish(msg)

                    # record ping end, and wait if we have to.
                    #ping_end = rospy.Time.now()
                    #ping_length = ping_end - ping_start

                    # wait a bit to retrigger on the PPS
                    #sleep_time = rospy.Duration(1.0) - ping_length - rospy.Duration(0.1)
                    #sleep_time.sleep()
                except Exception as e:
                    # rospy.logerr already has been called. Let's sleep on this
                    error_rate.sleep()

                #rospy.spin_once()  # XXX: This might not be necessary due to the process_messages() thread

        except serial.SerialException as e:
            rospy.logerr(f"Error while opening serial port {self.device_path}: {e}")
        finally:
            if self.device_file:
                self.device_file.close()


    def serial_read(self, sz):
        total_read = 0
        while total_read < sz:
            buf = self.device_file.read(sz - total_read)
            bytes_read = len(buf)

            if bytes_read > 0:
                total_read += bytes_read
            elif bytes_read == 0:
                # File end
                return bytes_read
            else:
                return -1

        return total_read

    def measure_depth(self, msg, data_points):
        depth = 0

        if not self.initial_command_sent:
            # Do not attempt to measure depth if the initial command has not been sent
            rospy.logerr("Initial command not sent. Cannot measure depth.")
            return depth

        # Read return data header
        hdr = self.device_file.read(12)

        if len(hdr) == 12:
            # Update last recived data
            self.last_data_received_time = rospy.Time.now()
            
            # Verify capabilities
            serial_status = struct.unpack("BB", hdr[2:4])[0]

            if not serial_status & 0x01:
                rospy.logerr("Echosounder not detected")

            # Verify that we support automatic trigger mode
            if not serial_status & 0x04:
                rospy.logerr("Automatic trigger mode not supported. Pings will be unsynchronized")

            if serial_status & 0x80:
                rospy.logerr("Character overrun detected")

            #current_time = rospy.Time.now()
            #msg.header.stamp = rospy.Time.now()
            #rospy.loginfo(f"Ping: {current_time.secs}.{current_time.nsecs}")
            #msg.header.stamp.nsecs = current_time.nsecs
            #header = Header()
            #header.stamp = rospy.Time.now()
            #msg.header = header

            # Read datapoints
            if data_points > 0:
                echo_data = self.device_file.read(data_points)
                if len(echo_data) != data_points:
                    rospy.logerr(f"Could not read datapoints ({len(echo_data)} bytes read)")
                    raise Exception("Error reading datapoints")

                # TODO: process datapoints

            # Read termination character
            termination_character = self.device_file.read(1)
            while termination_character != b'\xFC':
                termination_character = self.device_file.read(1)

            # Read profile range
            profile_range = self.device_file.read(2)
            profile_high, profile_low = struct.unpack("BB", profile_range)

            depth_centimeters = (profile_high << 8) | profile_low
            depth = float(depth_centimeters) / 100.0

        # Validate delay betwen now and last data recived
        elapsed_time = rospy.Time.now() - self.last_data_received_time
        if elapsed_time.to_sec() > 1.1:
            rospy.logwarn("Last data transmitted over 1 second.")

        return depth

if __name__ == "__main__":
    rospy.init_node("sonar_imagenex852")

    try:
        # TODO: parameterize path
        sonar = Imagenex852("/dev/sonar")
        sonar.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt Exception: {e}")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
