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
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


#include <tf2_ros/transform_listener.h>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"

#include "../../utils/QuaternionUtils.h"


typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;

class TelemetryServer {
public:
    TelemetryServer(): transformListener(buffer){
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&TelemetryServer::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&TelemetryServer::on_close,this,std::placeholders::_1));
	srv.set_message_handler(bind(&TelemetryServer::on_message,this,std::placeholders::_1,std::placeholders::_2));
        stateTopic = n.subscribe("state", 1000, &TelemetryServer::stateChanged,this);
	getLoggingStatusService = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
	toggleLoggingService = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
    }

    void on_message(connection_hdl hdl, server::message_ptr msg) {
	rapidjson::Document document;

        if(document.Parse(msg->get_payload().c_str()).HasParseError()){
        	//Not valid JSON
                return;
        }

        if(document.HasMember("command") && document["command"].IsString()){
        	//get command
                std::string command = document["command"].GetString();

                if(command.compare("getLoggingStatus")==0){
			bool isRecording = getRecordingStatus();
                	sendRecordingStatus(hdl,isRecording);
                }
                else if(command.compare("startLogging")==0){
			logger_service::ToggleLogging toggle;

			toggle.request.loggingEnabled = true;

			if(toggleLoggingService.call(toggle)){
				if(toggle.response.loggingStatus){
                			sendRecordingStatus(hdl,true);
				}
				else{
					ROS_ERROR("Failed at enabling logging");
				}
			}
			else{
				ROS_ERROR("Error while calling ToggleLogging service");
			}
               	}
                else if(command.compare("stopLogging")==0){
                        logger_service::ToggleLogging toggle;

                        toggle.request.loggingEnabled = false;

                        if(toggleLoggingService.call(toggle)){
                                if(!toggle.response.loggingStatus){
                                        sendRecordingStatus(hdl,false);
                                }
                                else{
                                        ROS_ERROR("Failed at disabling logging");
                                }
                        }
                        else{
                                ROS_ERROR("Error while calling ToggleLogging service");
                        }
                }
                else{
                	ROS_ERROR("Unknown command");
                }
        }
        else{
        	//no command found. ignore
                ROS_ERROR("No command found");
        }
    }

    void on_open(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.erase(hdl);
    }

    void sendRecordingStatus(connection_hdl & hdl, bool isRecording){
	rapidjson::Document document;
	document.SetObject();
	rapidjson::Value recordingStatus(rapidjson::Type::kObjectType);

	recordingStatus.AddMember("status",isRecording,document.GetAllocator());

	document.AddMember("recordingStatus",recordingStatus,document.GetAllocator());

        rapidjson::StringBuffer sb;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
        document.Accept(writer);
        std::string jsonString = sb.GetString();

        srv.send(hdl,jsonString,websocketpp::frame::opcode::text);
    }

    bool getRecordingStatus(){
	logger_service::GetLoggingStatus status;

	if(!getLoggingStatusService.call(status)){
		ROS_ERROR("Error while calling GetLoggingStatus service");
	}

	return status.response.status;
    }

    void convertState2json(const state_controller_msg::State & state, std::string & json) {
        rapidjson::Document document(rapidjson::kObjectType);
		rapidjson::Document telemetry(rapidjson::kObjectType);

        if( !state.position.header.seq  &&  state.position.status.status < 0) {
            //No fix
            rapidjson::Value positionArray(rapidjson::Type::kArrayType);
            telemetry.AddMember("position", positionArray, telemetry.GetAllocator()); // empty position array
        } else {
            rapidjson::Value positionArray(rapidjson::Type::kArrayType);
            rapidjson::Value longitude(state.position.longitude);
            rapidjson::Value latitude(state.position.latitude);

            positionArray.PushBack(longitude, telemetry.GetAllocator());
            positionArray.PushBack(latitude, telemetry.GetAllocator());
            telemetry.AddMember("position", positionArray, telemetry.GetAllocator());
        }

        if(!state.odom.header.seq){
            rapidjson::Value attitudeArray(rapidjson::Type::kArrayType);
            telemetry.AddMember("attitude", attitudeArray, telemetry.GetAllocator()); // empty attitude array
        }
	else {
	    try{
                double heading = 0;
                double pitch = 0;
                double roll = 0;

		geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));

		QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation,state.odom.pose.pose.orientation,heading,pitch,roll);

		rapidjson::Value attitudeArray(rapidjson::Type::kArrayType);
		rapidjson::Value headingValue(heading);
		rapidjson::Value pitchValue(pitch);
		rapidjson::Value rollValue(roll);

		attitudeArray.PushBack(headingValue, telemetry.GetAllocator());
		attitudeArray.PushBack(pitchValue, telemetry.GetAllocator());
		attitudeArray.PushBack(rollValue, telemetry.GetAllocator());
		telemetry.AddMember("attitude", attitudeArray, telemetry.GetAllocator());
	    }
	    catch (tf2::TransformException &ex) {
		ROS_WARN("IMU transform missing: %s",ex.what());
 	    }
        }

        if(!state.depth.header.seq){
            rapidjson::Value depthArray(rapidjson::Type::kArrayType);
            telemetry.AddMember("depth", depthArray, telemetry.GetAllocator()); // empty depth array
        }
	else {
            rapidjson::Value depthArray(rapidjson::Type::kArrayType);
            rapidjson::Value z(state.depth.point.z);

            depthArray.PushBack(z, telemetry.GetAllocator());
            telemetry.AddMember("depth", depthArray, telemetry.GetAllocator());
        }

        if(!state.vitals.header) {
            rapidjson::Value vitalsArray(rapidjson::Type::kArrayType);
            telemetry.AddMember("vitals", vitalsArray, telemetry.GetAllocator()); // empty vitals array
        }
	else{
            rapidjson::Value vitalsArray(rapidjson::Type::kArrayType);
            rapidjson::Value cputemp(state.vitals.cputemp);
            rapidjson::Value cpuload((int) state.vitals.cpuload);
            rapidjson::Value freehdd((int) state.vitals.freehdd);
            rapidjson::Value uptime((int) state.vitals.uptime);
            rapidjson::Value vbat(state.vitals.vbat);
            rapidjson::Value rh((int) state.vitals.rh);
            rapidjson::Value temp((int) state.vitals.temp);
            rapidjson::Value psi((int) state.vitals.psi);

            vitalsArray.PushBack(cputemp, telemetry.GetAllocator());
            vitalsArray.PushBack(cpuload, telemetry.GetAllocator());
            vitalsArray.PushBack(freehdd, telemetry.GetAllocator());
            vitalsArray.PushBack(uptime, telemetry.GetAllocator());
            vitalsArray.PushBack(vbat, telemetry.GetAllocator());
            vitalsArray.PushBack(rh, telemetry.GetAllocator());
            vitalsArray.PushBack(temp, telemetry.GetAllocator());
            vitalsArray.PushBack(psi, telemetry.GetAllocator());
            telemetry.AddMember("vitals", vitalsArray, telemetry.GetAllocator());
        }

        document.AddMember("telemetry", telemetry, document.GetAllocator());
        rapidjson::StringBuffer sb;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
        document.Accept(writer);
        json = sb.GetString();

    }

    void stateChanged(const state_controller_msg::State & state) {
        uint64_t timestamp = (state.odom.header.stamp.sec * 1000000) + (state.odom.header.stamp.nsec/1000);
        //TODO: maybe add our own header?
        if(timestamp - lastTimestamp > 200000){
            std::string jsonString;
            convertState2json(state, jsonString);

            std::lock_guard<std::mutex> lock(mtx);
            for (auto it : connections) {
                 srv.send(it,jsonString,websocketpp::frame::opcode::text);
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

    void stop() {
	    websocketpp::lib::error_code ec_stop_listening;
	    srv.stop_listening(ec_stop_listening);
	    if(ec_stop_listening) {
	        ROS_ERROR_STREAM("failed to stop listening: " << ec_stop_listening.message());
	        return;
	    }

	    std::string closingMessage = "Server has closed the connection";
	    std::lock_guard<std::mutex> lock(mtx); // server stopped listening is this needed?
	    for (auto it : connections) {
            	websocketpp::lib::error_code ec_close_connection;
            	srv.close(it,websocketpp::close::status::normal,closingMessage, ec_close_connection);
            	if(ec_close_connection) {
                	ROS_ERROR_STREAM("failed to close connection: " << ec_close_connection.message());
             	}
            }

	    ROS_INFO("Stopping Configuration server");
            srv.stop();
     }

private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server srv;
    con_list connections;
    std::mutex mtx;

    ros::NodeHandle n;
    ros::Subscriber stateTopic;
    ros::ServiceClient getLoggingStatusService;
    ros::ServiceClient toggleLoggingService;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transformListener;

    uint64_t lastTimestamp;
};

#endif
