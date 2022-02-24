
//#include <gnss_dummy/gnss_dummy.h>
//#include <ros/ros.h>
#include <gtest/gtest.h>
#include "hydroball_config_websocket/hydroball_config_websocket.h"

#include <thread>
//#include <chrono>

TEST(configWebsocket, configInit) {
	
	//ros::NodeHandle n;
	//ros::Subscriber sub2 = n.subscribe("speed", 10, speedReceived);
	std::string configFilePath ("/home/ubuntu/Poseidon/config.txt");

	ConfigurationServer server(configFilePath);
	uint16_t port = 9004;
	std::thread t(&ConfigurationServer::run,&server, port);
	
	ros::Rate loop_rate( 10 ); // 10 Hz
	int counter = 0;
	while(ros::ok()){
		ros::spinOnce();
        loop_rate.sleep();
        counter++;
        if (counter == 30){
        	server.stop();
        	t.join();
        	ros::shutdown();
        }
	}
		
	ASSERT_TRUE(true);
	//ROS_ERROR_STREAM("speed count : "<< speedCounter.getCount()<<"\n");
	//ASSERT_TRUE(speedCounter.getCount() == 10);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "configWebsocket");
    
    testing::InitGoogleTest(&argc, argv);
    
    //std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    //ros::shutdown();
    //t.join();
    
    return res;
}
