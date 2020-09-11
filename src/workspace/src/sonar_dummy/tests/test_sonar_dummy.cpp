#include <sonar_dummy/sonar_dummy.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

class SonarDummyTestSuite : public ::testing::Test {
  public:
    SonarDummyTestSuite() {
    }
    ~SonarDummyTestSuite() {}
};

//global variable to tests if callback was called by subscriber
bool subscriberReceivedData = false;
void callback_AssertSubscriberReceivedWhatIsPublished(const geometry_msgs::PointStamped & depthMsg)
{
    double expected_z = 11.2;

    double epsilon = 1e-15;
    ASSERT_NEAR(expected_z, depthMsg.point.z, epsilon) << "subscriber didn't receive expected depth";

    subscriberReceivedData = true;
}

TEST(SonarDummyTestSuite, testCaseSubscriberReceivedWhatIsPublished) {

    Sonar sonar;
    ros::NodeHandle nh;

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("depth", 1000, callback_AssertSubscriberReceivedWhatIsPublished);

    //publish a imu message
    double z = 11.2;
    sonar.message(z);

    //wait a bit for subscriber to pick up message
    sleep(1);

    //verify that callback was called by subscriber
    ASSERT_TRUE(subscriberReceivedData) << "callback was not called by subscriber";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestSonarNode");
    testing::InitGoogleTest(&argc, argv);
    std::thread t([]{while(ros::ok()) ros::spin();});
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    t.join();
    
    return res;
}
