#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>

#include <unistd.h>

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
/*
bool logging = false;

bool toggleLoggingCallback(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){
	ROS_INFO("test-toggle");
    logging = !logging;
    return true;
}


bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response){
    response.status = logging;
    return true;
}
*/
/*
bool getLoggingMode(logger_service::GetLoggingMode::Request &request, logger_service::GetLoggingMode::Response &response){
	response.loggingMode = loggingMode;
	return true;
}
	
bool setLoggingMode(logger_service::SetLoggingMode::Request &request, logger_service::SetLoggingMode::Response &response){
	loggingMode = request.loggingMode;
	return true;

}
*/

TEST(LoggerTextTestSuite, testToggle) {

    ros::NodeHandle n;

    // setup service servers
    //ros::ServiceServer getLoggingStatusServiceServer = n.advertiseService("get_logging_status_local", getLoggingStatus);
    //ros::ServiceServer toggleLoggingServiceServer    = n.advertiseService("toggle_logging_local", toggleLoggingCallback);
    
    // setup service clients
    ros::ServiceClient getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    ros::ServiceClient toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");

    ASSERT_TRUE(getLoggingStatusServiceClient.waitForExistence(ros::Duration(10.0)));
    ASSERT_TRUE(toggleLoggingServiceClient.waitForExistence(ros::Duration(10.0)));

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
	
    sleep(2);

    // toggle to disable logging
    toggle.request.loggingEnabled = false;

    toggleLoggingServiceClient.call(toggle);
    sleep(2);
    ASSERT_FALSE(toggle.response.loggingStatus) << "logging callback was not called by service server";
	
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging status was not changed after disable toggle";
	
}

TEST(LoggerTextTestSuite, testChangingLoggingMode) {

    ros::NodeHandle n;	
    ros::ServiceClient getLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
	ros::ServiceClient setLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");

	ASSERT_TRUE(getLoggingModeServiceClient.waitForExistence(ros::Duration(10.0)));
	ASSERT_TRUE(setLoggingModeServiceClient.waitForExistence(ros::Duration(10.0)));
	
	logger_service::SetLoggingMode newMode;
	newMode.request.loggingMode = 3;
	setLoggingModeServiceClient.call(newMode);
	sleep(2);
	logger_service::GetLoggingMode whatIsMode;
	getLoggingModeServiceClient.call(whatIsMode);
	sleep(2);
	ASSERT_TRUE(whatIsMode.response.loggingMode == 3);
	
	newMode.request.loggingMode = 1;
	setLoggingModeServiceClient.call(newMode);
	sleep(2);
	getLoggingModeServiceClient.call(whatIsMode);
	sleep(2);
	ASSERT_TRUE(whatIsMode.response.loggingMode == 1);
}
int main(int argc, char **argv) {

    ros::init(argc, argv, "TestLoggerService");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
