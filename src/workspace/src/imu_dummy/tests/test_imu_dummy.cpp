#include <imu_dummy/imu_dummy.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

#include <unistd.h>

#include <sensor_msgs/Imu.h>

#include "hydroball_config_websocket/hydroball_config_websocket.h"
#include "../../utils/QuaternionUtils.h"

#include "../../utils/Constants.hpp"

class ImuDummyTestSuite : public ::testing::Test {
  public:
    ImuDummyTestSuite() {
    }
    ~ImuDummyTestSuite() {}
};

/*
//global variable to tests if callback was called by subscriber
bool subscriberReceivedData = false;
void callback_AssertSubscriberReceivedWhatIsPublished(const nav_msgs::Odometry & imuMsg)
{
    double expected_yaw = 135.4;
    double expected_pitch = 2.3;
    double expected_roll = -1.4;

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;


    tf2::Matrix3x3 mat(imuMsg.pose.pose.orientation);
    mat.getEulerYPR(yaw,pitch,roll);

    double epsilon = 1e-15;
    ASSERT_NEAR(expected_yaw, yaw, epsilon) << "subscriber didn't receive expected yaw angle";
    ASSERT_NEAR(expected_pitch, pitch, epsilon) << "subscriber didn't receive expected pitch angle";
    ASSERT_NEAR(expected_roll, roll, epsilon) << "subscriber didn't receive expected roll angle";

    subscriberReceivedData = true;
}

void printAllTopics() {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for(unsigned int i=0; i<topic_infos.size(); i++) {
        std::cout << topic_infos[i].name << std::endl;
    }
}


TEST(ImuDummyTestSuite, testCaseSubscriberReceivedWhatIsPublished) {

    std::string configFilePath = "../../../../test/config4Tests.txt";
    ConfigurationServer configurationServer(configFilePath);
    uint16_t portConfig = 9004;
    //run the server in separate thread
    std::thread configurationThread(std::bind(&ConfigurationServer::run,&configurationServer, portConfig));

    IMU imu;
    ros::NodeHandle nh;

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("odom", 1000, callback_AssertSubscriberReceivedWhatIsPublished);

    //publish a imu message
    uint32_t sequenceNumber= 0;
    double yaw = 135.4;
    double pitch = 2.3;
    double roll = -1.4;
    imu.message(sequenceNumber, yaw, pitch, roll);

    //wait a bit for subscriber to pick up message
    sleep(1);

    //verify that callback was called by subscriber
    ASSERT_TRUE(subscriberReceivedData) << "callback was not called by subscriber";

    configurationServer.stop();
    configurationThread.join();
}
*/

TEST(ImuDummyTestSuite, testCaseQuaternionUtils) {

    double headingTest = 15;
    double pitchTest   = 30;
    double rollTest    = 45;

    double headingBoresight = 0;
    double pitchBoresight   = 0;
    double rollBoresight    = 0;

    tf2::Quaternion transform;
    transform.setRPY(D2R(rollBoresight), D2R(pitchBoresight), D2R(headingBoresight));
    geometry_msgs::Quaternion transformQ = tf2::toMsg(transform);

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ  = tf2::toMsg(pose);

    double expectedHeading = 0;
    double expectedPitch   = 0;
    double expectedRoll    = 0;

    // this one gives degrees
    QuaternionUtils::applyTransform(transformQ, poseQ, expectedHeading, expectedPitch, expectedRoll);

    double epsilon = 1e-12;
    ASSERT_NEAR(expectedHeading, headingTest, epsilon) << "wrong heading";
    ASSERT_NEAR(expectedPitch, pitchTest, epsilon) << "wrong pitch";
    ASSERT_NEAR(expectedRoll, rollTest, epsilon) << "wrong roll";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestImuDummyNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    t.join();
    
    return res;
}
