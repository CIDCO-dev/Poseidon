#ifndef sonar_dummy
#define sonar_dummy

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <setting_msg/ConfigurationService.h>
#include <mutex>

class Sonar{
	private:
		std::mutex mtx;

		ros::NodeHandle node;
		ros::Publisher sonarTopic;
		ros::ServiceClient configurationClient;

		uint32_t sequenceNumber;

	        uint8_t sonarStartGain = 0x06;
        	uint8_t sonarRange = 32;
        	uint8_t sonarAbsorbtion = 0x14; //20 = 0.2db    675kHz
        	uint8_t sonarPulseLength= 150;

	public:
		Sonar(){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
			configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
			getConfiguration();
		}

        	void message(double z);
		void run();

        	void getConfiguration(){
        		char *    configKeys[] = {"sonarStartGain","sonarRange","sonarAbsorbtion","sonarPulseLength"};
            		uint8_t * valuePtrs[]  = {&sonarStartGain,&sonarRange,&sonarAbsorbtion,&sonarPulseLength};

            		for(int i=0;i<4;i++){
                		std::string valueString = getConfigValue(configKeys[i]);
                		setConfigValue(valueString, valuePtrs[i]);
            		}
        	}

		std::string getConfigValue(std::string key) {
			setting_msg::ConfigurationService srv;

			srv.request.key = key;

			if(configurationClient.call(srv)) {
				return srv.response.value;
			}
			else {
				return "";
			}
		}

		void setConfigValue(const std::string & valStr,uint8_t * val) {
			mtx.lock();
			sscanf(valStr.c_str(),"%hhu",val);
			mtx.unlock();
		}

};

#endif
