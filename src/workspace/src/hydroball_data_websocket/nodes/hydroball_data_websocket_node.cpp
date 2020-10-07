#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "hydroball_data_websocket/hydroball_data_websocket.h"


int main(int argc,char ** argv){
    ros::init(argc,argv,"hydroball_websocket_controller");

    TelemetryServer server;
    uint16_t port = 9002;
    std::thread t(std::bind(&TelemetryServer::run,&server, port));


	ros::spin(); // loop until shutdown or ctrl-c

	server.stop(); // stop the server

	t.join(); // join the thread before returning from node

	return 0;
}
