#include "ros/ros.h"
#include "hydroball_config_websocket/hydroball_config_websocket.h"


int main(int argc,char ** argv){
	ros::init(argc,argv,"hydroball_config_websocket");

	if (argc!=2){
		ROS_ERROR("Missing output log path\n");
    		return 1;
	}

	std::string configFilePath (argv[1]);

	ConfigurationServer server(configFilePath);
	std::thread t(std::bind(&ConfigurationServer::receiveMessages,&server));
	server.run(9004);
}
