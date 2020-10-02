#include <hydroball_data_websocket/hydroball_data_websocket.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "state_controller_msg/State.h"
#include "sensor_msgs/NavSatFix.h"

//#include "../../utils/ClientWpp.hpp"
#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"

#include "ClientTester.hpp"

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <thread>
#include <chrono>

typedef websocketpp::client<websocketpp::config::asio_client> client;

/*
class TestClient : public ClientWpp {

public:
    TestClient(std::string & uri) : ClientWpp{uri} {}
    ~TestClient(){};

protected:
    void business() {
        while(1) {
            bool wait = false;

            { // check status scope
                scoped_lock guard(lock);
                if(done) {
                    break;
                }

                if(!open) {
                    wait = true; // wait until client opens connection
                }
            }

            if(wait) {
                sleep(1000);
                continue;
            }

            // Do client work here
        }
    }
};
*/

class DataWebsocketTestSuite : public ::testing::Test {
  public:
    DataWebsocketTestSuite() {
    }
    ~DataWebsocketTestSuite() {}
};

class HydroBallDataWebSocketTest : ClientTester {

public:

    //test values
    static constexpr double testLongitude = -68.5;
    static constexpr double testLatitude = 48.5;

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

    double epsilon = 1e-12;

    void on_open(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl) {
        // do nothing, this client only listens
    }

    void on_fail(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl) {
        ASSERT_TRUE(false) << "client connection failed during hydroball data websokect test";
    }

    void on_message(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl, websocketpp::config::asio_client::message_type::ptr msg) {
        ASSERT_TRUE(true) << "client received message from server";

        rapidjson::Document document;

        if(document.Parse(msg->get_payload().c_str()).HasParseError()){
        	ASSERT_TRUE(false) << "Couldn't parse json message";
            return;
        }

        if( ! document.HasMember("telemetry")) {
            ASSERT_TRUE(false) << "Message does not contain telemetry";
            return;
        } else {
            const rapidjson::Value& position = document["position"];
            ASSERT_NEAR(testLongitude, position[0].GetDouble(), epsilon) << "client didn't receive expected longitude";
            ASSERT_NEAR(testLatitude, position[1].GetDouble(), epsilon) << "client didn't receive expected latitude";

            const rapidjson::Value& attitude = document["attitude"];
            ASSERT_NEAR(testHeading, attitude[0].GetDouble(), epsilon) << "client didn't receive expected heading";
            ASSERT_NEAR(testPitch, attitude[1].GetDouble(), epsilon) << "client didn't receive expected pitch";
            ASSERT_NEAR(testRoll, attitude[2].GetDouble(), epsilon) << "client didn't receive expected roll";

            const rapidjson::Value& depth = document["depth"];
            ASSERT_NEAR(testDepth, depth[0].GetDouble(), epsilon) << "client didn't receive expected depth";

            const rapidjson::Value& vitals = document["vitals"];
            ASSERT_NEAR(test_cputemp, vitals[0].GetDouble(), epsilon) << "client didn't receive expected cputemp";
            ASSERT_NEAR(test_cpuload, vitals[0].GetDouble(), epsilon) << "client didn't receive expected cpuload";
            ASSERT_NEAR(test_freeram, vitals[0].GetDouble(), epsilon) << "client didn't receive expected freeram";
            ASSERT_NEAR(test_freehdd, vitals[0].GetDouble(), epsilon) << "client didn't receive expected freehdd";
            ASSERT_NEAR(test_uptime, vitals[0].GetDouble(), epsilon) << "client didn't receive expected uptime";
            ASSERT_NEAR(test_vbat, vitals[0].GetDouble(), epsilon) << "client didn't receive expected vbat";
            ASSERT_NEAR(test_rh, vitals[0].GetDouble(), epsilon) << "client didn't receive expected rh";
            ASSERT_NEAR(test_temp, vitals[0].GetDouble(), epsilon) << "client didn't receive expected temp";
            ASSERT_NEAR(test_psi, vitals[0].GetDouble(), epsilon) << "client didn't receive expected psi";
        }
    }

    void on_close(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl) {
        // do nothing
    }

};

void createNavMsg(double longitude, double latitude, double height, sensor_msgs::NavSatFix & msg) {
    msg.header.seq=0;
	msg.header.stamp=ros::Time::now();
	msg.status.service = 1;
	msg.header.stamp.nsec=0;
    msg.longitude=longitude;
	msg.latitude=latitude;
	msg.altitude=height;
}


void createOdomMsg(double yawDegrees, double pitchDegrees, double rollDegrees, nav_msgs::Odometry & msg) {
    msg.header.seq=0;
    msg.header.stamp=ros::Time::now();
    QuaternionUtils::convertDegreesToQuaternion(yawDegrees,pitchDegrees,rollDegrees,msg);
}

void createSonarMsg(double depth, geometry_msgs::PointStamped & msg) {
    msg.point.z = depth;
}

void createVitalsMsg(double cputemp, double cpuload, double freeram, double freehdd, double uptime, double vbat, double rh, double temp, double psi, raspberrypi_vitals_msg::sysinfo & msg) {
    //msg.header.seq=0;
    msg.cputemp = cputemp;
    msg.cpuload = cpuload;
    msg.freeram = freeram;
    msg.freehdd = freehdd;
    msg.uptime = uptime;
    msg.vbat = vbat;
    msg.rh = rh;
    msg.temp = temp;
    msg.psi = psi;
}


TEST(DataWebsocketTestSuite, testCaseSubscriberReceivedWhatIsPublished) {

    try {
        TelemetryServer telemetryServer;
        uint16_t port = 9002;
        std::thread telemetryThread(std::bind(&TelemetryServer::run,&telemetryServer,port));

        //wait for server to spinup
        sleep(5);


        //create a connection to the telemetryServer
        std::string uri = "ws://localhost:9002/";
        HydroBallDataWebSocketTest ct;
        client c;

        //no logging
        c.clear_access_channels(websocketpp::log::alevel::all);

        c.init_asio();

        //register handlers
        c.set_open_handler(websocketpp::lib::bind(&HydroBallDataWebSocketTest::on_open, &ct, &c, websocketpp::lib::placeholders::_1));
        c.set_fail_handler(websocketpp::lib::bind(&HydroBallDataWebSocketTest::on_fail, &ct, &c, websocketpp::lib::placeholders::_1));
        c.set_message_handler(websocketpp::lib::bind(&HydroBallDataWebSocketTest::on_message, &ct, &c, websocketpp::lib::placeholders::_1, websocketpp::lib::placeholders::_2));
        c.set_close_handler(websocketpp::lib::bind(&HydroBallDataWebSocketTest::on_close, &ct, &c, websocketpp::lib::placeholders::_1));

        // start the client's asio loop
        std::thread telemetryClientThread(std::bind(&client::run,&c));



        //connect client to telemetry server
        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        c.connect(con);

        //wait for client to spinup
        sleep(5);




        // create a publisher node to topic state
        ros::NodeHandle nh;
        ros::Publisher statePublisher = nh.advertise<state_controller_msg::State>("state", 1000);

        // publish a state message to "state"
        state_controller_msg::State state;
        memset(&state,0,sizeof(state));


        // position
        sensor_msgs::NavSatFix nav;
        memset(&nav,0,sizeof(nav));
        createNavMsg(HydroBallDataWebSocketTest::testLongitude, HydroBallDataWebSocketTest::testLatitude, HydroBallDataWebSocketTest::testHeight, nav);
        memcpy(&state.position,&nav,sizeof(nav));

        //odometry
        nav_msgs::Odometry odom;
        memset(&odom,0,sizeof(odom));
        createOdomMsg(HydroBallDataWebSocketTest::testHeading, HydroBallDataWebSocketTest::testPitch, HydroBallDataWebSocketTest::testRoll, odom);
        memcpy(&state.odom,&odom,sizeof(odom));

        //sonar
        geometry_msgs::PointStamped sonar;
        memset(&sonar,0,sizeof(sonar));
        createSonarMsg(HydroBallDataWebSocketTest::testDepth, sonar);
        memcpy(&state.depth,&sonar,sizeof(sonar));

        // vitals
        raspberrypi_vitals_msg::sysinfo vital;
        memset(&vital,0,sizeof(vital));
        createVitalsMsg(
            HydroBallDataWebSocketTest::test_cputemp,
             HydroBallDataWebSocketTest::test_cpuload,
              HydroBallDataWebSocketTest::test_freeram,
               HydroBallDataWebSocketTest::test_freehdd,
                HydroBallDataWebSocketTest::test_uptime,
                 HydroBallDataWebSocketTest::test_vbat,
                  HydroBallDataWebSocketTest::test_rh,
                   HydroBallDataWebSocketTest::test_temp,
                    HydroBallDataWebSocketTest::test_psi,
                     vital
        );
        memcpy(&state.vitals,&vital,sizeof(vital));



        std::cout << state << std::endl;
        // publish the state
        statePublisher.publish(state);

        // The handlers defined in



        //wait a bit for subscriber to pick up message
        sleep(10);

        telemetryServer.stop();
        telemetryThread.join();

        ASSERT_TRUE(true);

    }
    catch(...) {
        ADD_FAILURE() << "Uncaught exception";
    }
}

void callback_AssertSubscriberReceivedWhatIsPublished(const state_controller_msg::State & stateMsg)
{
    std::cout << "printing depth:" << std::endl;
    std::cout << stateMsg.depth.point.z << std::endl;
}

/*
TEST(DataWebsocketTestSuite, testAreWorking) {
    ASSERT_TRUE(true);

    std::cout << "Testing nodes" << std::endl;

    // create a publisher node to topic state
    ros::NodeHandle nh;
    ros::Publisher statePublisher = nh.advertise<state_controller_msg::State>("state", 1000);

    //setup subscriber with callback that will assert if test passes
    ros::Subscriber sub = nh.subscribe("state", 1000, callback_AssertSubscriberReceivedWhatIsPublished);

    // publish a state message to "state"
    std::cout << "building state" << std::endl;
    state_controller_msg::State state;
    memset(&state, 0, sizeof(state));

    //sonar


    geometry_msgs::PointStamped sonar;
    memset(&sonar, 0, sizeof(sonar));
    createSonarMsg(HydroBallDataWebSocketTest::testDepth, sonar);
    memcpy(&state.depth,&sonar,sizeof(sonar));

    //state.depth.point.z = HydroBallDataWebSocketTest::testDepth;

    std::cout << "publishing state" << std::endl;




    // publish the state
    statePublisher.publish(state);

    //wait for subscriber to receive
    sleep(1);


}
*/



int main(int argc, char** argv) {

    ros::init(argc, argv, "TestDataWebSocketNode");
    
    testing::InitGoogleTest(&argc, argv);


    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();

    t.join();
    
    return res;
}
