#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>

#include <unistd.h>

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"


class LoggerTextTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    LoggerTextTestSuite() {
    }
    ~LoggerTextTestSuite() {}
};

bool logging = false;
bool toggleLoggingCallback(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){
    logging = !logging;
    return true;
}


bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response){
    response.status = logging;
    return true;
}

TEST(LoggerTextTestSuite, testToggle) {

    ros::NodeHandle n;

    // setup service servers
    ros::ServiceServer getLoggingStatusServiceServer = n.advertiseService("get_logging_status", getLoggingStatus);
    ros::ServiceServer toggleLoggingServiceServer    = n.advertiseService("toggle_logging", toggleLoggingCallback);

    ASSERT_FALSE(logging) << "logging should not be enabled";

    // setup service clients
    ros::ServiceClient getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    ros::ServiceClient toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");

    // toggle to enable logging
    logger_service::ToggleLogging toggle;
    toggle.request.loggingEnabled = true;

    toggleLoggingServiceClient.call(toggle);
    sleep(2);
    ASSERT_TRUE(logging) << "logging callback was not called by service server";

    logger_service::GetLoggingStatus status;
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";

    sleep(2);

    // toggle to disable logging
    toggle.request.loggingEnabled = false;

    toggleLoggingServiceClient.call(toggle);
    sleep(2);
    ASSERT_FALSE(logging) << "logging callback was not called by service server";

    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging status was not changed after disable toggle";
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestLoggerNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}