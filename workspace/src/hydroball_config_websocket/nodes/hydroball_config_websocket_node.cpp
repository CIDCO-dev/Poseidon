#include "ros/ros.h"
#include "ros/console.h"
#include "hydroball_config_websocket/hydroball_config_websocket.h"


int main(int argc,char ** argv){
	ros::init(argc,argv,"hydroball_config_websocket");

	if (argc!=2){
		ROS_ERROR("Missing output log path\n");
        return 1;
	}

	std::string configFilePath (argv[1]);

	ConfigurationServer server(configFilePath);
	uint16_t port = 9004;
	//run the server in separate thread
	std::thread t(std::bind(&ConfigurationServer::run,&server, port));

    ros::Rate loop_rate( 10 ); // 10 Hz
	while(ros::ok()){
		ros::spinOnce();
        loop_rate.sleep();
	}

	server.stop(); // stop the server

	t.join(); // join the thread before returning from node

	return 0;
}
