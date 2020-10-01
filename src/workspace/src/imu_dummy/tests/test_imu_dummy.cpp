#include <imu_dummy/imu_dummy.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

#include <unistd.h>

#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.h"

class ImuDummyTestSuite : public ::testing::Test {
  public:
    ImuDummyTestSuite() {
    }
    ~ImuDummyTestSuite() {}
};

//global variable to tests if callback was called by subscriber
bool subscriberReceivedData = false;
void callback_AssertSubscriberReceivedWhatIsPublished(const nav_msgs::Odometry & imuMsg)
{
    double expected_yaw = D2R(135.4);
    double expected_pitch = D2R(2.3);
    double expected_roll = D2R(-1.4);

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    QuaternionUtils::convertToEulerAngles(imuMsg.pose.pose.orientation, yaw, pitch, roll);

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

    IMU imu;
    ros::NodeHandle nh;

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("odom", 1000, callback_AssertSubscriberReceivedWhatIsPublished);

    //publish a imu message
    uint32_t sequenceNumber= 0;
    double yaw = D2R(135.4);
    double pitch = D2R(2.3);
    double roll = D2R(-1.4);
    imu.message(sequenceNumber, yaw, pitch, roll);

    //wait a bit for subscriber to pick up message
    sleep(1);

    //verify that callback was called by subscriber
    ASSERT_TRUE(subscriberReceivedData) << "callback was not called by subscriber";
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
