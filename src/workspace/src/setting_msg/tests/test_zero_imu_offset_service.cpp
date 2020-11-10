#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <thread>
#include <unistd.h>
/*

#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "raspberrypi_vitals_msg/sysinfo.h"

*/
#include <map>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <random>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "state_controller_msg/GetStateService.h"
#include "setting_msg/ImuOffsetService.h"
#include "../../utils/Constants.hpp"

class ZeroImuOffsetTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    ZeroImuOffsetTestSuite() {
    }
    ~ZeroImuOffsetTestSuite() {}
};

void printAllTopics() {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for(unsigned int i=0; i<topic_infos.size(); i++) {
        std::string tname = topic_infos[i].name;
        ROS_ERROR(tname.c_str());
        //std::cout << topic_infos[i].name << std::endl;
    }
}

void getKeysAndValues(std::map<std::string,std::string> & m, std::vector<std::string> & keys, std::vector<std::string> & values) {
    for(std::map<std::string, std::string>::iterator it = m.begin(); it != m.end(); it++) {
        keys.push_back(it->first);
        ROS_ERROR(it->first.c_str());
        values.push_back(it->second);
        ROS_ERROR(it->second.c_str());
    }
}

void readConfigurationFromFile(std::string & configFilePath, std::map<std::string,std::string> & configuration){
		std::ifstream in;
		in.open(configFilePath);

		if(in.is_open()){
			std::string line;
			while(std::getline(in,line)){
				std::stringstream ss(line);
				std::string key;
				std::string value;

				ss >> key >> value;

				if(key.size() > 0 && value.size() > 0) {
					configuration[key]=value;
				}
			}

			in.close();
		}
		else{
			throw std::invalid_argument(std::string("Cannot open file ") + configFilePath);
		}
	}

TEST(ZeroImuOffsetTestSuite, testZeroImuOffset) {
    ros::NodeHandle nh;

    std::string packagePath = ros::package::getPath("setting_msg");
    std::string configFileOnTestPath = packagePath + "/tests/zeroImuOffsetTestConfig.txt";

    std::map<std::string,std::string> configurationBeforeTest;
    readConfigurationFromFile(configFileOnTestPath, configurationBeforeTest);

    ros::Publisher imuPublisher = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000); // this is the topic name in state_controller

    while( imuPublisher.getNumSubscribers() < 1 ) { // wait for subscriber in state controller to connect to publisher
        ros::spinOnce();
        sleep(1);
    }

    // publish imu data
	sensor_msgs::Imu imuMsg;
	uint32_t msgSequenceNumber = 42;
    imuMsg.header.seq=msgSequenceNumber;
    imuMsg.header.stamp=ros::Time::now();

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0,1);

    // change state in state controller with a new attitude
    double testRoll = dist(e2) * 15; // between 1 and 15
    double testPitch = dist(e2) * 15 + 15; // between 15 and 30
    double testHeading = dist(e2) * 15 + 30; // between 30 and 45

    tf2::Quaternion q;
    q.setRPY(D2R(testRoll),D2R(testPitch),D2R(testHeading));
    imuMsg.orientation = tf2::toMsg(q);
    imuPublisher.publish(imuMsg);

    sleep(2); // give enough time for StateController's state to be updated


    // register client service
    ros::ServiceClient zeroImuOffsetClient = nh.serviceClient<setting_msg::ImuOffsetService>("zero_imu_offsets");

    setting_msg::ImuOffsetService srv;
    zeroImuOffsetClient.call(srv);
    //ASSERT_TRUE() << "zero_imu_offsets service call returned false";
    
    std::map<std::string,std::string> configurationAfterTest;
    readConfigurationFromFile(configFileOnTestPath, configurationAfterTest);

    double expectedHeadingOffset = std::stod(configurationAfterTest["headingOffset"]);
    double expectedPitchOffset = std::stod(configurationAfterTest["pitchOffset"]);
    double expectedRollOffset = std::stod(configurationAfterTest["rollOffset"]);

    double epsilon = 1e-6; //currently file is written up to 6 digits
    ASSERT_NEAR(expectedHeadingOffset, -testHeading, epsilon) << "zero_imu_offsets service didn't set negative of heading";
    ASSERT_NEAR(expectedPitchOffset, -testPitch, epsilon) << "zero_imu_offsets service didn't set negative of pitch";
    ASSERT_NEAR(expectedRollOffset, -testRoll, epsilon) << "zero_imu_offsets service didn't set negative of roll";
}


int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "TestZeroImuOffsetService");

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}