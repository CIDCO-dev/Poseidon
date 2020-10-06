#include <hydroball_data_websocket/hydroball_data_websocket.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unistd.h>

#include "hydroball_config_websocket/hydroball_config_websocket.h"

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
            const rapidjson::Value& telemetry = document["telemetry"];

            if(! telemetry.HasMember("position")) {
                ASSERT_TRUE(false) << "Message does not contain position";
            } else {
                const rapidjson::Value& position = telemetry["position"];
                ASSERT_TRUE(position.IsArray()) << "position is not an array";
                ASSERT_NEAR(testLongitude, position[0].GetDouble(), epsilon) << "client didn't receive expected longitude";
                ASSERT_NEAR(testLatitude, position[1].GetDouble(), epsilon) << "client didn't receive expected latitude";
            }

            if(! telemetry.HasMember("attitude")) {
                ASSERT_TRUE(false) << "Message does not contain attitude";
            } else {
                const rapidjson::Value& attitude = telemetry["attitude"];
                ASSERT_TRUE(attitude.IsArray()) << "attitude is not an array";
                ASSERT_NEAR(testHeading, attitude[0].GetDouble(), epsilon) << "client didn't receive expected heading";
                ASSERT_NEAR(testPitch, attitude[1].GetDouble(), epsilon) << "client didn't receive expected pitch";
                ASSERT_NEAR(testRoll, attitude[2].GetDouble(), epsilon) << "client didn't receive expected roll";
            }

            if(! telemetry.HasMember("depth")) {
                ASSERT_TRUE(false) << "Message does not contain depth";
            } else {
                const rapidjson::Value& depth = telemetry["depth"];
                ASSERT_TRUE(depth.IsArray()) << "depth is not an array";
                ASSERT_NEAR(testDepth, depth[0].GetDouble(), epsilon) << "client didn't receive expected depth";
            }

            if(! telemetry.HasMember("vitals")) {
                ASSERT_TRUE(false) << "Message does not contain vitals";
            } else {
                const rapidjson::Value& vitals = telemetry["vitals"];
                ASSERT_TRUE(vitals.IsArray()) << "vitals is not an array";
                ASSERT_NEAR(test_cputemp, vitals[0].GetDouble(), epsilon) << "client didn't receive expected cputemp";
                ASSERT_NEAR(test_cpuload, vitals[1].GetDouble(), epsilon) << "client didn't receive expected cpuload";
                ASSERT_NEAR(test_freeram, vitals[2].GetDouble(), epsilon) << "client didn't receive expected freeram";
                ASSERT_NEAR(test_freehdd, vitals[3].GetDouble(), epsilon) << "client didn't receive expected freehdd";
                ASSERT_NEAR(test_uptime, vitals[4].GetDouble(), epsilon) << "client didn't receive expected uptime";
                ASSERT_NEAR(test_vbat, vitals[5].GetDouble(), epsilon) << "client didn't receive expected vbat";
                ASSERT_NEAR(test_rh, vitals[6].GetDouble(), epsilon) << "client didn't receive expected rh";
                ASSERT_NEAR(test_temp, vitals[7].GetDouble(), epsilon) << "client didn't receive expected temp";
                ASSERT_NEAR(test_psi, vitals[8].GetDouble(), epsilon) << "client didn't receive expected psi";
            }
        }
    }

    void on_close(websocketpp::client<websocketpp::config::asio_client> * c, websocketpp::connection_hdl hdl) {
        // do nothing
    }

};


TEST(DataWebsocketTestSuite, testCaseSubscriberReceivedWhatIsPublished) {

    try {
        //create configuration server
        std::string configFilePath = "../../../../config4Tests.txt";
        ConfigurationServer configurationServer(configFilePath);
        uint16_t portConfig = 9004;
        //run the server in separate thread
        std::thread configurationThread(std::bind(&ConfigurationServer::run,&configurationServer, portConfig));
        //wait for server to spinup
        sleep(2);
        std::cout << "Config server running" << std::endl;


        // create telemetry server
        TelemetryServer telemetryServer;
        uint16_t portTelemetry = 9002;
        std::thread telemetryThread(std::bind(&TelemetryServer::run,&telemetryServer,portTelemetry));

        //wait for server to spinup
        sleep(2);
        std::cout << "telemetry server running" << std::endl;


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
        std::cout << "client thread running" << std::endl;



        //connect client to telemetry server
        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        c.connect(con);

        //wait for client to spinup
        sleep(2);
        std::cout << "client connected to server" << std::endl;




        // create a publisher node to topic state
        ros::NodeHandle nh;
        ros::Publisher statePublisher = nh.advertise<state_controller_msg::State>("state", 1000);

        // publish a state message to "state"
        state_controller_msg::State state;
        //memset(&state,0,sizeof(state));


        // position
        state.position.header.seq = 1;
        state.position.header.stamp=ros::Time::now();
        state.position.status.status = 1;
        state.position.header.stamp.nsec=0;
        state.position.longitude=HydroBallDataWebSocketTest::testLongitude;
        state.position.latitude=HydroBallDataWebSocketTest::testLatitude;

        //odometry
        state.odom.header.seq=1;
        state.odom.header.stamp=ros::Time::now();
        tf2::Quaternion q;
        q.setRPY(D2R(HydroBallDataWebSocketTest::testRoll),D2R(HydroBallDataWebSocketTest::testPitch),D2R(HydroBallDataWebSocketTest::testHeading));
        state.odom.pose.pose.orientation = tf2::toMsg(q);

        //sonar
        state.depth.header.seq = 1;
        state.depth.header.stamp=ros::Time::now();
        state.depth.point.z = HydroBallDataWebSocketTest::testDepth;

        // vitals
        state.vitals.header = 1;
        state.vitals.cputemp = HydroBallDataWebSocketTest::test_cputemp;
        state.vitals.cpuload = HydroBallDataWebSocketTest::test_cpuload;
        state.vitals.freeram = HydroBallDataWebSocketTest::test_freeram;
        state.vitals.freehdd = HydroBallDataWebSocketTest::test_freehdd;
        state.vitals.uptime = HydroBallDataWebSocketTest::test_uptime;
        state.vitals.vbat = HydroBallDataWebSocketTest::test_vbat;
        state.vitals.rh = HydroBallDataWebSocketTest::test_rh;
        state.vitals.temp = HydroBallDataWebSocketTest::test_temp;
        state.vitals.psi = HydroBallDataWebSocketTest::test_psi;



        std::cout << state << std::endl;
        // publish the state
        statePublisher.publish(state);
        std::cout << "state published" << std::endl;

        //wait a bit for subscriber to pick up message
        sleep(10);

        telemetryServer.stop();

        configurationServer.stop();


        telemetryThread.join();
        configurationThread.join();

        telemetryClientThread.join();

        ASSERT_TRUE(true);

    }
    catch(...) {
        ADD_FAILURE() << "Uncaught exception";
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TestDataWebSocketNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();

    t.join();
    
    return res;
}
