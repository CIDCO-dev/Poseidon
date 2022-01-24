#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <filesystem>
#include <unistd.h>
#include "logger_text/logger_text.h"
#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

class LoggerTextTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    LoggerTextTestSuite() {
    }
    ~LoggerTextTestSuite() {}
  	
};

TEST(LoggerTextTestSuite, testGeneratingFiles) {
	ros::NodeHandle n;
	std::string outPath = "/home/ubuntu/unittestPoseidonRecord";
    Writer writer(outPath); 
	ros::Subscriber sub1 = n.subscribe("fix", 1, &Writer::gnssCallback,&writer); //without gpx fix -> no logging
	
	// setup service servers
    ros::ServiceServer getLoggingStatusServiceServer = n.advertiseService("get_logging_status", &Writer::getLoggingStatus,&writer);
    ros::ServiceServer toggleLoggingServiceServer    = n.advertiseService("toggle_logging", &Writer::toggleLogging,&writer);
	
    // setup service clients
    ros::ServiceClient getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    ros::ServiceClient toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");

    logger_service::GetLoggingStatus status;
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging should not be enabled";
	sleep(2);
	// toggle to enable logging
    logger_service::ToggleLogging toggle;
    toggle.request.loggingEnabled = true;

    toggleLoggingServiceClient.call(toggle);
    sleep(2);
    ASSERT_TRUE(toggle.response.loggingStatus) << "logging callback was not called by service server";
	
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";
	
	sleep(5);
	
    toggle.request.loggingEnabled = false;
    toggleLoggingServiceClient.call(toggle);
    sleep(2);
    //ASSERT_FALSE(status.response.status) << "logging should be stoped"; 
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging status was not changed after enable toggle";
    sleep(10); 
    
    
    int numberOfFiles = 0;
    std::filesystem::path PATH = outPath;
    std::string gpsFile;
		for(auto &dir_entry: std::filesystem::directory_iterator{PATH}){
		    if(dir_entry.is_regular_file()){
		    	std::string temp = std::filesystem::canonical(dir_entry);
		    	if (temp.find("_gnss.txt")){
		    		gpsFile = temp;
		    	}
		    	numberOfFiles++;
		    }
		}
    
    ASSERT_TRUE(numberOfFiles == 3);
    
    
    int numberOfLines = 0;
    std::string line;
    std::ifstream myfile(gpsFile);

    while (std::getline(myfile, line)){
    	numberOfLines++;
    }
    //ROS_ERROR_STREAM ("Number of lines in text file: " << numberOfLines);
    ASSERT_TRUE(numberOfLines >=7); //on slower pc number of lines might be lower
    
    
    
    
    //clean folder
    std::filesystem::path PATH2 = outPath;
    for(auto &dir_entry: std::filesystem::directory_iterator{PATH2}){
	    if(dir_entry.is_regular_file()){
		    numberOfFiles--;
		    std::filesystem::remove(dir_entry);
	    }
	}
    ASSERT_TRUE(numberOfFiles == 0);
    
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "TestLoggerText");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
