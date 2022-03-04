/*
 * Copyright (c) 2014, Peter Thorson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the WebSocket++ Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/*
modification and test made by: Patrick Charron-Morneau
date: 04-03-2022
*/

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
	std::map<std::string, std::string> originalConfig{{"headingOffset", "0"},{"headingOffset", "0"}, {"pitchOffset", "0"},
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
	sleep(1);
	
	std::map<std::string, std::string> testConfiguration;
	testConfig.getConfigMap(testConfiguration);
	
	//ROS_ERROR_STREAM("nb element: " << testConfiguration.size());
	ASSERT_TRUE(testConfiguration.size() == 9);
	int valid = 0;
	for(const auto& [key, value]: originalConfig){
		if(testConfiguration.contains(key) && testConfiguration[key] == value ){
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
