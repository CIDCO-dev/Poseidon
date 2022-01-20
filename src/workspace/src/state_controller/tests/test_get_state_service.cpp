#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <unistd.h>

#include "state_controller_msg/GetStateService.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "raspberrypi_vitals_msg/sysinfo.h"
#include "../../utils/Constants.hpp"

class GetStateServiceTestSuite : public ::testing::Test {
  //in case we want some setup, teardown
  public:

    GetStateServiceTestSuite() {
    }
    ~GetStateServiceTestSuite() {}
};

class Get_State_Test {

public:

    //test values
    static constexpr double testLongitude = -68.5;
    static constexpr double testLatitude = 48.5;
    static constexpr double testHeight = 29.1;

    static constexpr double testRoll = 45.0;
    static constexpr double testPitch = 30.0;
    static constexpr double testHeading = 15.0;

    static constexpr double testDepth = 10.1;

    static constexpr double test_cputemp = 1.0;
    static constexpr double test_cpuload = 2.0;
    static constexpr double test_freeram = 3.0;
    static constexpr double test_freehdd = 4.0;
    static constexpr double test_uptime = 5.0;
    static constexpr double test_vbat = 6.0;
    static constexpr double test_rh = 7.0;
    static constexpr double test_temp = 8.0;
    static constexpr double test_psi = 9.0;

    static constexpr double epsilon = 1e-12;
};

void printAllTopics() {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for(unsigned int i=0; i<topic_infos.size(); i++) {
        std::string tname = topic_infos[i].name;
        ROS_ERROR(tname.c_str());
        //std::cout << topic_infos[i].name << std::endl;
    }
}

TEST(GetStateServiceTestSuite, testGetState) {

    ros::NodeHandle nh;

    // create publishers for state_controller
    ros::Publisher positionPublisher  = nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);
    ros::Publisher imuPublisher = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000); // this is the topic name in state_controller
    ros::Publisher sonarPublisher = nh.advertise<geometry_msgs::PointStamped>("depth", 1000);
    ros::Publisher vitalsPublisher = nh.advertise<raspberrypi_vitals_msg::sysinfo>("vitals", 1000);

    // since subscribers declared in StateController exist before these publishers are declared,
    // we have to wait until ROS figures this out
    while(  positionPublisher.getNumSubscribers() < 1 &&
            imuPublisher.getNumSubscribers() < 1 &&
            sonarPublisher.getNumSubscribers() < 1 &&
            vitalsPublisher.getNumSubscribers() < 1
    ) {
        ros::spinOnce();
        sleep(1);
    }

    // publish position
    sensor_msgs::NavSatFix navMsg;
    uint32_t msgSequenceNumber = 12;
	navMsg.header.seq=msgSequenceNumber;
	navMsg.header.stamp=ros::Time::now();
	navMsg.status.service = 1;
	navMsg.header.stamp.nsec=0;
	navMsg.longitude=Get_State_Test::testLongitude;
	navMsg.latitude=Get_State_Test::testLatitude;
	navMsg.altitude=Get_State_Test::testHeight;
	positionPublisher.publish(navMsg);

	// publish imu data
	sensor_msgs::Imu imuMsg;
    imuMsg.header.seq=msgSequenceNumber;
    imuMsg.header.stamp=ros::Time::now();

    tf2::Quaternion q;
    q.setRPY(D2R(Get_State_Test::testRoll),D2R(Get_State_Test::testPitch),D2R(Get_State_Test::testHeading));
    imuMsg.orientation = tf2::toMsg(q);
    imuPublisher.publish(imuMsg);

	// publish sonar data
	geometry_msgs::PointStamped sonarMsg;
	sonarMsg.header.seq=msgSequenceNumber;
    sonarMsg.header.stamp=ros::Time::now();
    sonarMsg.point.z = Get_State_Test::testDepth;
    sonarPublisher.publish(sonarMsg);

	// publish vitals
	raspberrypi_vitals_msg::sysinfo vitalMsg;
	//vitalMsg.header = 1; //header is of Header type object
    vitalMsg.cputemp = Get_State_Test::test_cputemp;
    vitalMsg.cpuload = Get_State_Test::test_cpuload;
    vitalMsg.freeram = Get_State_Test::test_freeram;
    vitalMsg.freehdd = Get_State_Test::test_freehdd;
    vitalMsg.uptime = Get_State_Test::test_uptime;
    vitalMsg.vbat = Get_State_Test::test_vbat;
    vitalMsg.rh = Get_State_Test::test_rh;
    vitalMsg.temp = Get_State_Test::test_temp;
    vitalMsg.psi = Get_State_Test::test_psi;
    vitalsPublisher.publish(vitalMsg);

	sleep(2); // give enough time for StateController's state to be updated

    // register client service
    ros::ServiceClient getStateClient = nh.serviceClient<state_controller_msg::GetStateService>("get_state");

    // Call service
    state_controller_msg::GetStateService srv;
    if(getStateClient.call(srv)) { // this is the tested unit

        //postion
        ASSERT_NEAR(Get_State_Test::testLongitude, srv.response.state.position.longitude, Get_State_Test::epsilon) << "get_state service didn't return expected longitude";
        ASSERT_NEAR(Get_State_Test::testLatitude, srv.response.state.position.latitude, Get_State_Test::epsilon) << "get_state service didn't return expected latitude";
        ASSERT_NEAR(Get_State_Test::testHeight, srv.response.state.position.altitude, Get_State_Test::epsilon) << "get_state service didn't return expected height";

        //attitude -- go through tf2
        double heading;
        double pitch;
        double roll;

        tf2::Quaternion q;
        tf2::fromMsg(srv.response.state.imu.orientation, q);
        tf2::Matrix3x3 mat(q);
        mat.getEulerYPR(heading,pitch,roll);
        ASSERT_NEAR(Get_State_Test::testHeading, R2D(heading), Get_State_Test::epsilon) << "get_state service didn't return expected heading";
        ASSERT_NEAR(Get_State_Test::testPitch, R2D(pitch), Get_State_Test::epsilon) << "get_state service didn't return expected pitch";
        ASSERT_NEAR(Get_State_Test::testRoll, R2D(roll), Get_State_Test::epsilon) << "get_state service didn't return expected roll";

        //sonar
        ASSERT_NEAR(Get_State_Test::testDepth, srv.response.state.depth.point.z, Get_State_Test::epsilon) << "get_state service didn't return expected depth";

        //vitals
        ASSERT_NEAR(Get_State_Test::test_cputemp, srv.response.state.vitals.cputemp, Get_State_Test::epsilon) << "get_state service didn't return expected cputemp";
        ASSERT_NEAR(Get_State_Test::test_cpuload, srv.response.state.vitals.cpuload, Get_State_Test::epsilon) << "get_state service didn't return expected cpuload";
        ASSERT_NEAR(Get_State_Test::test_freeram, srv.response.state.vitals.freeram, Get_State_Test::epsilon) << "get_state service didn't return expected freeram";
        ASSERT_NEAR(Get_State_Test::test_freehdd, srv.response.state.vitals.freehdd, Get_State_Test::epsilon) << "get_state service didn't return expected freehdd";
        ASSERT_NEAR(Get_State_Test::test_uptime, srv.response.state.vitals.uptime, Get_State_Test::epsilon) << "get_state service didn't return expected uptime";
        ASSERT_NEAR(Get_State_Test::test_vbat, srv.response.state.vitals.vbat, Get_State_Test::epsilon) << "get_state service didn't return expected vbat";
        ASSERT_NEAR(Get_State_Test::test_rh, srv.response.state.vitals.rh, Get_State_Test::epsilon) << "get_state service didn't return expected rh";
        ASSERT_NEAR(Get_State_Test::test_temp, srv.response.state.vitals.temp, Get_State_Test::epsilon) << "get_state service didn't return expected temp";
        ASSERT_NEAR(Get_State_Test::test_psi, srv.response.state.vitals.psi, Get_State_Test::epsilon) << "get_state service didn't return expected psi";
    } else {
        ASSERT_TRUE(false) << " get_state service call returned false";
    }
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "TestGetStateService");

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
