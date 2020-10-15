#include "ros/ros.h"
#include "hydroball_files_websocket/hydroball_files_websocket.h"


int main(int argc,char ** argv){
    ros::init(argc,argv,"hydroball_files_websocket");
    std::string logPath (argv[1]);

    if (logPath.length()<2){
		ROS_INFO("Missing output log path\n");
    		return 1;
	}
    //logPath = logPath + "*";
 
    //ROS_INFO("Using log path at %s",logPath.c_str());
    ControlFiles server(logPath);
    uint16_t port = 9003;
    std::thread t(std::bind(&ControlFiles::run,&server, port));

    ros::spin(); // loop until shutdown or ctrl-c

    server.stop(); // stop the server

	t.join(); // join the thread before returning from node
}
