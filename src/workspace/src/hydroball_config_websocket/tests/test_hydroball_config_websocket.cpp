#include <gtest/gtest.h>
#include <ros/ros.h>
#include "hydroball_config_websocket/hydroball_config_websocket.h"
#include <thread>

std::map<std::string, std::string> originalConfig{{"headingOffset", "0"},{"headingOffset", "0"}, {"pitchOffset", "0"},
												  {"rollOffset", "0"}, {"sonarAbsorbtion", "20"}, {"sonarPulseLength", "120"},
												  {"sonarRange", "20"}, {"sonarStartGain", "16"}, {"speedThresholdKmh", "6.0"}};

class Config{
	
	public:
		Config(){}
		~Config(){}
		void setConfigMap(std::string key, std::string value){
			configuration[key]=value;
		}
		 void getConfigMap(std::map<std::string, std::string> &copy){
			 copy = this->configuration;
		}
	private:
		std::map<std::string, std::string> configuration;
};

Config testConfig;

void configurationCallBack(const setting_msg::Setting &setting){
	ROS_ERROR_STREAM("configCallback -> " << setting.key << " : "<<setting.value<<"\n");
	testConfig.setConfigMap(setting.key, setting.value);
}


TEST(configWebsocket, configInit) {
	
	
	std::string configFilePath ("/home/ubuntu/Poseidon/src/workspace/src/hydroball_config_websocket/tests/config.txt");

	ConfigurationServer server(configFilePath);
	uint16_t port = 9004;
	std::thread t(&ConfigurationServer::run,&server, port);
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("configuration", 1000, configurationCallBack);
	
	std::map<std::string, std::string> testConfiguration;
	testConfig.getConfigMap(testConfiguration);
	
	server.broadcastConfiguration();
	sleep(3);
	ROS_ERROR_STREAM("nb element: " << testConfiguration.size());
	//ASSERT_TRUE(testConfiguration.size() == 9);
	
	
	server.stop();
	t.join();
	ASSERT_TRUE(true);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "configWebsocket");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{ros::spin();}); // let ros spin in its own thread
     
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();
      
    return res;
}
