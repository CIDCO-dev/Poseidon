#include <gtest/gtest.h>
#include <ros/ros.h>
#include "hydroball_config_websocket/hydroball_config_websocket.h"
#include <thread>


class Config{
	
	public:
		Config(){}
		~Config(){}
		void setConfigMap(std::string key, std::string value){
			this->configuration[key]=value;
		}
		 void getConfigMap(std::map<std::string, std::string> &copy){
			 copy = this->configuration;
		}
		void configurationCallBack(const setting_msg::Setting &setting){
			//ROS_ERROR_STREAM("configCallback -> " << setting.key << " : "<<setting.value<<"\n");
			setConfigMap(setting.key, setting.value);
		}
		
	private:
		std::map<std::string, std::string> configuration;
};

TEST(configWebsocket, configInit) {
	std::map<std::string, std::string> originalConfig{{"headingOffset", "0"}, {"loggingMode", "1"},{"pitchOffset", "0"},
												  {"rollOffset", "0"}, {"sonarAbsorbtion", "20"}, {"sonarPulseLength", "120"},
												  {"sonarRange", "20"}, {"sonarStartGain", "16"}, {"speedThresholdKmh", "6.0"}};
	
	Config testConfig;
	
	std::string configFilePath ("/home/ubuntu/Poseidon/src/workspace/src/hydroball_config_websocket/tests/config.txt");

	ConfigurationServer server(configFilePath);
	uint16_t port = 9004;
	std::thread t(&ConfigurationServer::run,&server, port);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("configuration", 1000, &Config::configurationCallBack, &testConfig );
	
	server.broadcastConfiguration();
	sleep(2);
	
	std::map<std::string, std::string> testConfiguration;
	testConfig.getConfigMap(testConfiguration);

	ASSERT_TRUE(testConfiguration.size() == 9) << "config received:" << testConfiguration.size();
	int valid = 0;
	int wtf =0;
	for (const auto& kv : originalConfig) {
		wtf++;
		//ROS_ERROR_STREAM(wtf);
		ROS_ERROR_STREAM(kv.first << " : " << kv.second);
		auto result = testConfiguration.find(kv.first);
		if(result->first == kv.first && result->second == kv.second){
			valid++;
		}
	}
	ASSERT_TRUE(valid==9);

	server.stop();
	t.join();
	ASSERT_TRUE(true);
}

TEST(configWebsocket, ImuTransform) {
	ASSERT_FALSE(true) << "broadcastImuTransform() and saveConfigutation() unit tests not done";
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
