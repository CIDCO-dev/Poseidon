#include <gnss_dummy/gnss_dummy.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

#include <unistd.h>

class GnssDummyTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    GnssDummyTestSuite() {
    }
    ~GnssDummyTestSuite() {}
};

//global variable to tests if callback was called by subscriber
bool subscriberReceivedData = false;
void callback_AssertSubscriberReceivedWhatIsPublished(const sensor_msgs::NavSatFix& gnss)
{
    double epsilon = 1e-15;

    double expectedLongitude = 49.00;
    double expectedLatitude = 60.00;

    ASSERT_NEAR(expectedLongitude, gnss.longitude, epsilon);
    ASSERT_NEAR(expectedLatitude, gnss.latitude, epsilon);

    subscriberReceivedData = true;
}

TEST(GnssDummyTestSuite, testCaseSubscriberReceivedWhatIsPublished) {
    GNSS gnss;
    ros::NodeHandle nh;

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("fix", 1, callback_AssertSubscriberReceivedWhatIsPublished);

    //publish a gnss message
    uint32_t sequenceNumber= 0;
    double longitude = 49.00;
    double latitude = 60.00;
    int status = 0; // status field required by current GNSS::message signature
    gnss.message(sequenceNumber, longitude, latitude, status);

    //wait a bit for subscriber to pick up message
    sleep(1);

    //verify that callback was called by subscriber
    ASSERT_TRUE(subscriberReceivedData);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "TestGnssDummyNode");
    testing::InitGoogleTest(&argc, argv);
    std::thread t([]{while(ros::ok()) ros::spin();});
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    t.join(); //t needs to join main thread otherwise it will raise exception and get a core dump
    return res;
}
