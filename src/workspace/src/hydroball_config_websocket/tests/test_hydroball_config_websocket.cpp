
//#include <gnss_dummy/gnss_dummy.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "hydroball_config_websocket/hydroball_config_websocket.h"

#include <thread>
//#include <chrono>

TEST(configWebsocket, configInit) {
	
	//ros::NodeHandle n;
	//ros::Subscriber sub2 = n.subscribe("speed", 10, speedReceived);
	std::string configFilePath (argv[1]);

	ConfigurationServer server(configFilePath);
	uint16_t port = 9004;
	
	ASSERT_TRUE(true);
	//ROS_ERROR_STREAM("speed count : "<< speedCounter.getCount()<<"\n");
	//ASSERT_TRUE(speedCounter.getCount() == 10);
}

int main(int argc, char** argv) {
    
    //ros::init(argc, argv, "configWebsocket");
    
    testing::InitGoogleTest(&argc, argv);
    
    //std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    //ros::shutdown();
    //t.join();
    
    return res;
}
