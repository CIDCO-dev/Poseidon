#include "ros/ros.h"
#include "sonar_nmea_0183_tcp_client/sonar_nmea_0183_tcp_client.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "nmea_device");

	std::string device;

	//if no params present. use default values of /dev/sonar
	/* in launch file , private parameter in parameter server
	<node pkg="sonar_nmea_0183_tcp_client" type="nmea_device_node" name="Sonar" output="screen" respawn="true" respawn_delay="1">
		<param name="device" type="str" value="/dev/ttyUSB0"/>
	</node>
	*/
	if(!ros::param::get("/Sonar/device", device)){
		device = "/dev/sonar";
	}

	//TODO: get useDepth/usePOsition/useAttitude from parameters
	DeviceNmeaClient nmea(device,true,false,false);
	nmea.run();
}

