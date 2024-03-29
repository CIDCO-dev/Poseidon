#include <ros/ros.h>
#include <gtest/gtest.h>

#include <fstream> 
#include <filesystem>
#include <unistd.h>
#include "loggerText.h"

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

class LoggerTextTestSuite : public ::testing::Test {
  public:

    LoggerTextTestSuite() {}
    ~LoggerTextTestSuite() {}
    
    protected:
		ros::NodeHandle n;
		ros::ServiceClient getLoggingStatusServiceClient;
		ros::ServiceClient toggleLoggingServiceClient;
		ros::ServiceClient GetLoggingModeServiceClient;
		ros::ServiceClient SetLoggingModeServiceClient;
		std::string outPath;
    	
    	virtual void SetUp() override{
    		this->outPath = "/home/ubuntu/unittestPoseidonRecord";
    		// setup service clients
    		this->getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    		this->toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
    		this->GetLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
    		this->SetLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");
    	}
    	
    	virtual void TearDown() override{
    		std::filesystem::remove_all(outPath);
    	}
  	
};

TEST_F(LoggerTextTestSuite, testGeneratingFiles) {

	std::filesystem::create_directory(outPath);
	LoggerText logger(outPath);
	
	logger_service::GetLoggingStatus status;
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging should not be enabled";
	
	// wait for gnss dummy node
	sleep(2);
    
	// toggle to enable logging
    logger_service::ToggleLogging toggle;
    toggle.request.loggingEnabled = true;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(toggle.response.loggingStatus) << "logging callback was not called by service server";
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";
    
    // try toggle to stop logging
    toggle.request.loggingEnabled = false;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(status.response.status) << "logging should not be stoped"; 
    
	
	// because theres no config file, logging mode is set to always on and cannot be toogle off
	logger_service::SetLoggingMode newMode;
	newMode.request.loggingMode = 2; // manual
	SetLoggingModeServiceClient.call(newMode);
	logger_service::GetLoggingMode whatIsMode;
	GetLoggingModeServiceClient.call(whatIsMode);
	ASSERT_TRUE(whatIsMode.response.loggingMode == 2);
	
	// toggle to stop logging
    toggle.request.loggingEnabled = false;
    toggleLoggingServiceClient.call(toggle);
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging status was not changed after enable toggle";
    
    // toggle to enable logging
    toggle.request.loggingEnabled = true;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(toggle.response.loggingStatus) << "logging callback was not called by service server";
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";
    logger.finalize();
    
    int numberOfFiles = 0;
    std::filesystem::path PATH = outPath;
    std::string gpsFile;
		for(auto &dir_entry: std::filesystem::directory_iterator{PATH}){
		    if(dir_entry.is_regular_file()){
//		    	std::string temp = std::filesystem::canonical(dir_entry);
//		    	if (temp.find("_gnss.txt")){
//		    		gpsFile = temp;
//		    	}
		    	numberOfFiles++;
		    }
		}
    
    ASSERT_TRUE(numberOfFiles == 4);
    
    
//    int numberOfLines = 0;
//    std::string line;
//    std::ifstream myfile(gpsFile);

//    while (std::getline(myfile, line)){
//    	numberOfLines++;
//    }
//    ROS_ERROR_STREAM ("Number of lines in text file: " << numberOfLines);
//    ASSERT_TRUE(numberOfLines >=7); //on slower pc number of lines might be lower
	
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
