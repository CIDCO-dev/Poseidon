#ifndef hydroball_data_websocket
#define hydroball_data_websocket

#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <glob.h>
#include <iostream>
#include <istream>
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>

#include "ros/ros.h"

#include "state_controller_msg/State.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "../../utils/QuaternionUtils.h"


typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;

class TelemetryServer {
public:
    TelemetryServer(){
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&TelemetryServer::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&TelemetryServer::on_close,this,std::placeholders::_1));
	srv.set_message_handler(bind(&TelemetryServer::on_message,this,std::placeholders::_1,std::placeholders::_2));
        stateTopic = n.subscribe("state", 1000, &TelemetryServer::stateChanged,this);
    }


    void on_message(connection_hdl hdl, server::message_ptr msg) {

    }

    void on_open(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.erase(hdl);
    }

    void stateChanged(const state_controller_msg::State & state) {

        uint64_t timestamp = (state.odom.header.stamp.sec * 1000000) + (state.odom.header.stamp.nsec/1000);
        std::string str;
        if(
                //TODO: maybe add our own header?
                timestamp - lastTimestamp > 200000
        ){
            //Build JSON object to send to web interface
            std::stringstream ss;
            ss << "{";

            if( !state.position.header.seq  &&  state.position.status.status < 0){
                //No fix
                ss << "\"position\":[],";
            }
            else{
                ss << "\"position\":[" << std::setprecision(12) << state.position.longitude << "," <<  state.position.latitude  << "],";
            }

            if(!state.odom.header.seq){
                ss << "\"attitude\":[],";
            }
            else{
                double heading = 0;
                double pitch = 0;
                double roll = 0;

                QuaternionUtils::convertToEulerAngles(state.odom.pose.pose.orientation,heading,pitch,roll);

                ss << "\"attitude\":[" << std::setprecision(5)  << R2D(heading) << "," << R2D(pitch) << "," << R2D(roll)  << "],";
            }

            if(!state.depth.header.seq){
                ss << "\"depth\":[],";
            }
            else{
                ss << "\"depth\":[" << std::setprecision(6) << state.depth.point.z  << "],";
            }

	    if(!state.vitals.header){
                ss << "\"vitals\":[],";
            }
            else{
              ss << "\"vitals\":[" << std::setprecision(5)  << state.vitals.cputemp << "," << (int) state.vitals.cpuload << "," << (int) state.vitals.freeram  << "," << (int) state.vitals.freehdd << "," << (int) state.vitals.uptime  << "," <<  state.vitals.vbat << "," << (int) state.vitals.rh  << "," << (int) state.vitals.temp << "," << (int) state.vitals.psi << "]";
            }

            ss << "}";
            std::lock_guard<std::mutex> lock(mtx);
            for (auto it : connections) {
                 srv.send(it,ss.str(),websocketpp::frame::opcode::text);
            }

            lastTimestamp = timestamp;
        }
    }

    void receiveMessages(){
        ros::spin();
    }

    void run(uint16_t port){
        srv.listen(port);
        srv.start_accept();
        srv.run();
    }

private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server srv;
    con_list connections;
    std::mutex mtx;

    ros::NodeHandle n;
    ros::Subscriber stateTopic;
    uint64_t lastTimestamp;
};

#endif
