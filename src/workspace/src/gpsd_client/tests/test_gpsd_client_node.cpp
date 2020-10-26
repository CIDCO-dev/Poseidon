#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <unistd.h>

#include "sensor_msgs/NavSatFix.h"

//global variable to test if callback was called by subscriber
bool subscriberReceivedData = false;
void callback_AssertSubscriberReceivedData(const sensor_msgs::NavSatFix & gnssMsg)
{
    if(!subscriberReceivedData) {
        subscriberReceivedData = true;
    }

    return;
}

TEST(GpsdClientTestSuite, testCaseSubscriberReceivedData) {

    ros::NodeHandle nh;

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("fix", 1000, callback_AssertSubscriberReceivedData);

    // wait for subscriber to receive data from imu
    sleep(10);

    //verify that callback was called by subscriber
    ASSERT_TRUE(subscriberReceivedData) << "callback was not called by subscriber";
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "TestGpsdClientNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}