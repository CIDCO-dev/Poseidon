#include "ros/ros.h"
#include "sonar_nmea_0183_tcp_client/sonar_nmea_0183_tcp_client.h"

int main(int argc,char** argv){
	bool useDepth = true;
	bool usePosition = false;
	bool useAttitude = false;
	
	ros::init(argc, argv, "nmea_device");

	std::string device;

	//if no params present. use default values of /dev/sonar
	if(!ros::param::get("/Sonar/device", device)){
		device = "/dev/sonar";
	}
	
	if(!ros::param::get("/Sonar/useDepth", useDepth)){
		useDepth = true;
	}
	
	if(!ros::param::get("/Sonar/usePosition", usePosition)){
		usePosition = false;
	}
	
	if(!ros::param::get("/Sonar/useAttitude", useAttitude)){
		useAttitude = false;
	}
	
	DeviceNmeaClient nmea(device,useDepth,usePosition,useAttitude);
	nmea.run();
}

