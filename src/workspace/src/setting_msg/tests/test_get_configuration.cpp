#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <unistd.h>

#include <setting_msg/ConfigurationService.h>

class GetConfigurationTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    GetConfigurationTestSuite() {
    }
    ~GetConfigurationTestSuite() {}
};

TEST(GetConfigurationTestSuite, testGetConfiguration) {
    double epsilon = 1e-12;

    sleep(2); // wait for config websocket to register service server

    // register client service
    ros::NodeHandle nh;
    ros::ServiceClient configurationClient = nh.serviceClient<setting_msg::ConfigurationService>("get_configuration");

    setting_msg::ConfigurationService srv;

    // test get heading offset
    srv.request.key = "headingOffset";
    std::string expectedHeadingOffset = "180"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedHeadingOffset == srv.response.value) << "get_configuration service didn't return expected headingOffset";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get pitch offset
    srv.request.key = "pitchOffset";
    std::string expectedPitchOffset = "10"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedPitchOffset == srv.response.value) << "get_configuration service didn't return expected pitchOffset";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get roll offset
    srv.request.key = "rollOffset";
    std::string expectedRollOffset = "15"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedRollOffset == srv.response.value) << "get_configuration service didn't return expected rollOffset";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get sonarAbsorbtion
    srv.request.key = "sonarAbsorbtion";
    std::string expectedSonarAbsorbtion = "20"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedSonarAbsorbtion == srv.response.value) << "get_configuration service didn't return expected sonarAbsorbtion";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get sonarPulseLength
    srv.request.key = "sonarPulseLength";
    std::string expectedSonarPulseLength = "120"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedSonarPulseLength == srv.response.value) << "get_configuration service didn't return expected sonarPulseLength";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get sonarRange
    srv.request.key = "sonarRange";
    std::string expectedSonarRange = "32"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedSonarRange == srv.response.value) << "get_configuration service didn't return expected sonarRange";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }

    // test get sonarStartGain
    srv.request.key = "sonarStartGain";
    std::string expectedSonarStartGain = "10"; // from config.txt

    if(configurationClient.call(srv)) {
        ASSERT_TRUE(expectedSonarStartGain == srv.response.value) << "get_configuration service didn't return expected sonarStartGain";
    } else {
        ASSERT_TRUE(false) << " get configuration service call returned false";
    }
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "TestGetConfigurationService");

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}