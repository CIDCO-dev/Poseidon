#ifndef MAIN_CPP
#define MAIN_CPP

#include <functional>
#include <mutex>
#include <set>
#include <thread>

#include "ros/ros.h"

#include "state_controller/State.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;

//TODO: move this to a util class

#define D2R(x) (x * ((double)180/(double)M_PI))

void convertToEulerAngles(const geometry_msgs::Quaternion & q,double & heading,double & pitch, double & roll){
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1){
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else{
        pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    heading = std::atan2(siny_cosp, cosy_cosp);
}

class ControlServer {
public:
    ControlServer() {
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&ControlServer::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlServer::on_close,this,std::placeholders::_1));
        
        stateTopic = n.subscribe("state", 1000, &ControlServer::stateChanged,this);
    }

    void on_open(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.erase(hdl);
    }
    
    
    void stateChanged(const state_controller::State & state) {
        
        uint64_t timestamp = (state.attitude.header.stamp.sec * 1000000) + (state.attitude.header.stamp.nsec/1000);
        
        if(
                //TODO: maybe add our own header?
                (timestamp - lastTimestamp > 200000)
                //&&
                //()
        ){
            //Build JSON object to send to web interface
            std::stringstream ss;
            ss << "{";

            if( !state.position.header.seq  &&  state.position.status.status < 0){
                //No fix
                ss << "\"position\":[],";
            }
            else{
                ss << "\"position\":[" << std::setprecision(12) << (int) state.position.longitude << "," <<  state.position.latitude  << "],";
            }

            if(!state.attitude.header.seq){
                ss << "\"attitude\":[],";
            }
            else{
                double heading = 0;
                double pitch = 0;
                double roll = 0;

                convertToEulerAngles(state.attitude.orientation,heading,pitch,roll);

                ss << "\"attitude\":[" << std::setprecision(5)  << D2R(heading) << "," << D2R(pitch) << "," << D2R(roll)  << "],";
            }

            if(!state.depth.header.seq){
                ss << "\"depth\":[]";
            }
            else{
                ss << "\"depth\":[" << std::setprecision(6) << state.depth.point.z  << "]";
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

int main(int argc,char ** argv){
    ros::init(argc,argv,"hydroball_websocket_controller");
    ControlServer server;
    std::thread t(std::bind(&ControlServer::receiveMessages,&server));
    server.run(9002);
}

#endif
