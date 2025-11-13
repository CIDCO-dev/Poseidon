#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

#include <unistd.h>

#include <sensor_msgs/Imu.h>

#include "hydroball_config_websocket/hydroball_config_websocket.h"
#include "../../utils/QuaternionUtils.h"

#include "../../utils/Constants.hpp"

class TestQuaternionUtils : public QuaternionUtils {

public:
    static double test_flipHeading(double headingDegrees) {
        return flipHeading(headingDegrees);
    }

    static double test_modulo360(double angleDegrees) {
	    return modulo360(angleDegrees);
	}

    static void test_quaternion2YPR(tf2::Quaternion q, double & headingDegrees,double & pitchDegrees,double & rollDegrees) {
        quaternion2YPR(q, headingDegrees, pitchDegrees, rollDegrees);
    }

    static void test_body2Enu(geometry_msgs::Quaternion & transform,geometry_msgs::Quaternion & pose, tf2::Quaternion & result) {
        body2Enu(transform, pose, result);
    }
};

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
*/

void printAllTopics() {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for(unsigned int i=0; i<topic_infos.size(); i++) {
        std::cout << topic_infos[i].name << std::endl;
    }
}


TEST(ImuDummyTestSuite, testCaseApplyTransformWithNoTransform) {

    double epsilon = 1e-12;

    double headingTest = 15;
    double pitchTest   = 30;
    double rollTest    = 45;

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ = tf2::toMsg(pose);

    double headingBoresight = 0;
    double pitchBoresight   = 0;
    double rollBoresight    = 0;

    tf2::Quaternion transform;
    transform.setRPY(D2R(rollBoresight), D2R(pitchBoresight), D2R(headingBoresight));
    geometry_msgs::Quaternion transformQ = tf2::toMsg(transform);

    double headingDegrees = 0.0;
    double pitchDegrees = 0.0;
    double rollDegrees = 0.0;
    QuaternionUtils::applyTransform(transformQ, poseQ, headingDegrees, pitchDegrees, rollDegrees);

    ASSERT_NEAR(headingDegrees, headingTest, epsilon) << "wrong heading";
    ASSERT_NEAR(pitchDegrees, pitchTest, epsilon) << "wrong pitch";
    ASSERT_NEAR(rollDegrees, rollTest, epsilon) << "wrong roll";
}

TEST(ImuDummyTestSuite, testCaseBody2ENU_15dHeading_90dHeadingBoresight) {

    double epsilon = 1e-12;

    double headingTest = 15;
    double pitchTest   = 0;
    double rollTest    = 0;

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ = tf2::toMsg(pose);

    double headingBoresight = 90;
    double pitchBoresight   = 0;
    double rollBoresight    = 0;

    tf2::Quaternion transform;
    transform.setRPY(D2R(rollBoresight), D2R(pitchBoresight), D2R(headingBoresight));
    geometry_msgs::Quaternion transformQ = tf2::toMsg(transform);

    double headingDegrees = 0.0;
    double pitchDegrees = 0.0;
    double rollDegrees = 0.0;
    QuaternionUtils::applyTransform(transformQ, poseQ, headingDegrees, pitchDegrees, rollDegrees);

    ASSERT_NEAR(headingDegrees, headingTest - headingBoresight, epsilon) << "wrong heading";
    ASSERT_NEAR(pitchDegrees, 0, epsilon) << "wrong pitch";
    ASSERT_NEAR(rollDegrees, 0, epsilon) << "wrong roll";
}


TEST(ImuDummyTestSuite, testCaseBody2ENU_15dPitch_90dHeadingBoresight) {

    double epsilon = 1e-12;

    double headingTest = 0;
    double pitchTest   = 15;
    double rollTest    = 0;

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ = tf2::toMsg(pose);

    double headingBoresight = 90;
    double pitchBoresight   = 0;
    double rollBoresight    = 0;

    tf2::Quaternion transform;
    transform.setRPY(D2R(rollBoresight), D2R(pitchBoresight), D2R(headingBoresight));
    geometry_msgs::Quaternion transformQ = tf2::toMsg(transform);

    double headingDegrees = 0.0;
    double pitchDegrees = 0.0;
    double rollDegrees = 0.0;
    QuaternionUtils::applyTransform(transformQ, poseQ, headingDegrees, pitchDegrees, rollDegrees);

    ROS_INFO_STREAM("heading: " << headingDegrees);
    ROS_INFO_STREAM("pitch: " << pitchDegrees);
    ROS_INFO_STREAM("roll: " << rollDegrees);

    ASSERT_NEAR(headingDegrees, -headingBoresight, epsilon) << "wrong heading";
    ASSERT_NEAR(pitchDegrees, rollTest, epsilon) << "wrong pitch";
    ASSERT_NEAR(rollDegrees, -pitchTest, epsilon) << "wrong roll";
}



TEST(ImuDummyTestSuite, testCaseBody2ENU_15dRoll_90dHeadingBoresight) {

    double epsilon = 1e-12;

    double headingTest = 0;
    double pitchTest   = 0;
    double rollTest    = 15;

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ = tf2::toMsg(pose);

    double headingBoresight = 90;
    double pitchBoresight   = 0;
    double rollBoresight    = 0;

    tf2::Quaternion transform;
    transform.setRPY(D2R(rollBoresight), D2R(pitchBoresight), D2R(headingBoresight));
    geometry_msgs::Quaternion transformQ = tf2::toMsg(transform);

    double headingDegrees = 0.0;
    double pitchDegrees = 0.0;
    double rollDegrees = 0.0;
    QuaternionUtils::applyTransform(transformQ, poseQ, headingDegrees, pitchDegrees, rollDegrees);

    ROS_INFO_STREAM("heading: " << headingDegrees);
    ROS_INFO_STREAM("pitch: " << pitchDegrees);
    ROS_INFO_STREAM("roll: " << rollDegrees);

    ASSERT_NEAR(headingDegrees, -headingBoresight, epsilon) << "wrong heading";
    ASSERT_NEAR(pitchDegrees, rollTest, epsilon) << "wrong pitch";
    ASSERT_NEAR(rollDegrees, -pitchTest, epsilon) << "wrong roll";
}



TEST(ImuDummyTestSuite, testCaseQuaternionToYPR) {

    double epsilon = 1e-12;

    double headingTest = 15;
    double pitchTest   = 30;
    double rollTest    = 45;

    tf2::Quaternion pose;
    pose.setRPY(D2R(rollTest), D2R(pitchTest), D2R(headingTest));
    geometry_msgs::Quaternion poseQ  = tf2::toMsg(pose);

    double headingDegrees = 0.0;
    double pitchDegrees = 0.0;
    double rollDegrees = 0.0;
    TestQuaternionUtils::test_quaternion2YPR(pose, headingDegrees, pitchDegrees, rollDegrees);

    ASSERT_NEAR(headingDegrees, headingTest, epsilon) << "wrong heading";
    ASSERT_NEAR(pitchDegrees, pitchTest, epsilon) << "wrong pitch";
    ASSERT_NEAR(rollDegrees, rollTest, epsilon) << "wrong roll";
}




TEST(ImuDummyTestSuite, testCaseModuloAngles) {

    double epsilon = 1e-12;

    double angle0 = 0;
    double angle0Mod360 = TestQuaternionUtils::test_modulo360(angle0);
    ASSERT_NEAR(angle0Mod360, 0, epsilon) << "wrong angleMod360: 0 should become 0";

    double angle360 = 360;
    double angle360Mod360 = TestQuaternionUtils::test_modulo360(angle360);
    ASSERT_NEAR(angle360Mod360, 0, epsilon) << "wrong angleMod360: 360 should become 0";

    double angleM360 = -360;
    double angleM360Mod360 = TestQuaternionUtils::test_modulo360(angleM360);
    ASSERT_NEAR(angleM360Mod360, 0, epsilon) << "wrong angleMod360: -360 should become 0";

    double angle719 = 719;
    double angle719Mod360 = TestQuaternionUtils::test_modulo360(angle719);
    ASSERT_NEAR(angle719Mod360, 359, epsilon) << "wrong angleMod360: 719 should become 359";

    double angleM719 = -719;
    double angleM719Mod360 = TestQuaternionUtils::test_modulo360(angleM719);
    ASSERT_NEAR(angleM719Mod360, 1, epsilon) << "wrong angleMod360: -719 should become 1";
}


TEST(ImuDummyTestSuite, testCaseFlipHeading) {

    double epsilon = 1e-12;

    double heading = 350;
    double flippedHeading = TestQuaternionUtils::test_flipHeading(heading);
    ASSERT_NEAR(flippedHeading, 10, epsilon) << "wrong flipped heading for 350";

    double headingM10 = -10;
    double flippedHeadingM10 = TestQuaternionUtils::test_flipHeading(headingM10);
    ASSERT_NEAR(flippedHeading, 10, epsilon) << "wrong flipped heading for -10";


    double heading0 = 0;
    double flippedHeading0 = TestQuaternionUtils::test_flipHeading(heading0);
    ASSERT_NEAR(flippedHeading0, 0, epsilon) << "wrong flipped heading for 0";

    double heading360 = 360;
    double flippedHeading360 = TestQuaternionUtils::test_flipHeading(heading360);
    ASSERT_NEAR(flippedHeading360, 0, epsilon) << "wrong flipped heading for 360";
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
