#include "ros/ros.h"
#include "hydroball_data_websocket/hydroball_data_websocket.h"


int main(int argc,char ** argv){
    ros::init(argc,argv,"hydroball_websocket_controller");

    TelemetryServer server;
    std::thread t(std::bind(&TelemetryServer::receiveMessages,&server));
    server.run(9002);
}
