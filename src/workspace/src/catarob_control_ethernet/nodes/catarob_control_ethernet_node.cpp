#include "ros/ros.h"
#include "catarob_control_ethernet/catarob_control_ethernet.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "catarob");

	std::string addr;
	int port;

	//if no params present. use default values of 127.0.0.1:5000
	if(!ros::param::get("ip_address", addr)){
		addr = "127.0.0.1";
	}

	if(!ros::param::get("port", port)){
		port = 5000;
	}

	int a,b,c,d;

	//verify IP address
	if(
		sscanf(addr.c_str(),"%d.%d.%d.%d",&a,&b,&c,&d)==4 &&
		a > 0 && a <= 255 &&
		b >= 0 && b <= 255 &&
		c >= 0 && c <= 255 &&
		d >= 0 && d <= 255 
	){

		if(
			port > 0 && port <= 65535
		){
			//TODO: get useDepth/usePOsition/useAttitude from parameters
			CATAROB catarob(argv[1],argv[2]);
			catarob.run();
		}
		else{
			std::cerr << "Bad TCP port: " << port << std::endl;
		}
	}
	else{
		std::cerr << "Bad IP address: " << addr << std::endl;
	}
}
