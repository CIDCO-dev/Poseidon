#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "diagnostics/diagnostics_websocket.h"


int main(int argc,char ** argv){
    ros::init(argc,argv,"diagnostics");

    DiagnosticsServer server;
    uint16_t port = 9099;
    std::thread t(std::bind(&DiagnosticsServer::run,&server, port));
	
	ros::spin(); // loop until shutdown or ctrl-c

	server.stop(); // stop the server

	t.join(); // join the thread before returning from node

	return 0;
}
