#include "ros/ros.h"
#include "hydroball_data_websocket/hydroball_data_websocket.h"


int main(int argc,char ** argv){
    ros::init(argc,argv,"hydroball_websocket_controller");
    std::string logPath (argv[1]);
    if (logPath.length()<2){
		ROS_INFO("Missing output log path\n");
    		return 1;
	}
    logPath = logPath + "*";
    
    //ROS_INFO("Using log path at %s",logPath.c_str());
    ControlServer server(logPath);
    std::thread t(std::bind(&ControlServer::receiveMessages,&server));
    server.run(9002);
}
