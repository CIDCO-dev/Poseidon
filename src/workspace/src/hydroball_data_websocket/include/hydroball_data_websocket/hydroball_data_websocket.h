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
#include <cstdio>
#include <algorithm>
#include <fstream>
#include <boost/lexical_cast.hpp>

#include "ros/ros.h"

#include "state_controller_msg/State.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "gnss_status_msg/GnssDiagnostic.h"


#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


#include <tf2_ros/transform_listener.h>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

#include "../../utils/QuaternionUtils.h"


typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;

class TelemetryServer {
public:
	TelemetryServer(): transformListener(buffer){
		srv.init_asio();
		srv.set_reuse_addr(true);
		srv.clear_access_channels(websocketpp::log::alevel::all);  //remove cout logging

		srv.set_open_handler(bind(&TelemetryServer::on_open,this,std::placeholders::_1));
		srv.set_close_handler(bind(&TelemetryServer::on_close,this,std::placeholders::_1));
		srv.set_message_handler(bind(&TelemetryServer::on_message,this,std::placeholders::_1,std::placeholders::_2));
		stateTopic = n.subscribe("state", 1000, &TelemetryServer::stateChanged,this);
		getLoggingStatusService = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
		toggleLoggingService = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
		getLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
		setLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");
		getLoggingStatusService.waitForExistence();
		gnssStatusSub = n.subscribe("gnss_status", 10, &TelemetryServer::onGnssStatusReceived, this);

	}

	void broadcastMessage(const std::string& message) {
		std::lock_guard<std::mutex> lock(mtx);
		for (auto& hdl : connections) {
			srv.send(hdl, message, websocketpp::frame::opcode::text);
		}
	}
	

	void onGnssStatusReceived(const gnss_status_msg::GnssDiagnostic::ConstPtr& msg) {
		rapidjson::Document doc;
		doc.SetObject();
		rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
	
		doc.AddMember("type", "gnss_status", allocator);
		doc.AddMember("fix_type", msg->fix_type, allocator);
		doc.AddMember("diff_soln", msg->diff_soln, allocator);
		doc.AddMember("carr_soln", msg->carr_soln, allocator);
		doc.AddMember("num_sv", msg->num_sv, allocator);
		doc.AddMember("h_acc", msg->horizontal_accuracy, allocator);
		doc.AddMember("v_acc", msg->vertical_accuracy, allocator);
	
		rapidjson::StringBuffer buffer;
		rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
		doc.Accept(writer);
	
		broadcastMessage(buffer.GetString());
	}
	

	void on_message(connection_hdl hdl, server::message_ptr msg) {
	rapidjson::Document document;
		if(document.Parse(msg->get_payload().c_str()).HasParseError()){
			//Not valid JSON
			ROS_ERROR("invalid json : %s",msg->get_payload().c_str() );
				return;
		}

		if(document.HasMember("command") && document["command"].IsString()){
			//get command
				std::string command = document["command"].GetString();
								  //getLoggingStatus
				if(command.compare("getLoggingInfo")==0){
					bool isRecording = getRecordingStatus();
					int loggingMode = getLoggingMode();
					sendRecordingInfo(hdl,isRecording, loggingMode);
				}
				else if(command.compare("startLogging")==0){
					logger_service::ToggleLogging toggle;

					toggle.request.loggingEnabled = true;

					if(toggleLoggingService.call(toggle)){
						if(toggle.response.loggingStatus){
							int loggingMode = getLoggingMode();
							sendRecordingInfo(hdl,true, loggingMode);
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
										int loggingMode = getLoggingMode();
										sendRecordingInfo(hdl,false, loggingMode);
								}
								else{
										ROS_ERROR("Failed at disabling logging");
								}
						}
						else{
								ROS_ERROR("Error while calling ToggleLogging service");
						}
				}
				/*
				else if(command.compare("setLoggingMode") == 0){
					std::cout<<"get wrecked ! \n";
				}
				*/
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
	
	void sendRecordingInfo(connection_hdl & hdl,bool isRecording, int loggingMode){
		rapidjson::Document document;
		document.SetObject();
		rapidjson::Value recordingStatus(rapidjson::Type::kObjectType);
		recordingStatus.AddMember("status",isRecording,document.GetAllocator());
		document.AddMember("recordingInfo",recordingStatus,document.GetAllocator());
		rapidjson::Value LoggingMode(rapidjson::Type::kObjectType);
		LoggingMode.AddMember("the_mode_is",loggingMode,document.GetAllocator());
		document.AddMember("loggingMode",LoggingMode,document.GetAllocator());
		
		rapidjson::StringBuffer sb;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
		document.Accept(writer);
		std::string jsonString = sb.GetString();
		srv.send(hdl,jsonString,websocketpp::frame::opcode::text);
	}
	
	int getLoggingMode(){
		logger_service::GetLoggingMode mode;
		if(!getLoggingModeServiceClient.call(mode)){
			ROS_ERROR("Error while calling GetLoggingMode service");
		}
		return  mode.response.loggingMode;
	}
	
	bool getRecordingStatus(){
	logger_service::GetLoggingStatus status;

	if(!getLoggingStatusService.call(status)){
		ROS_ERROR("Error while calling GetLoggingStatus service");
	}

	return status.response.status;
	}

	void refreshWifiInfo(){
		// refresh at most every 5 seconds to avoid hammering the system
		const double now = ros::Time::now().toSec();
		if(wifiHasData && (now - lastWifiQuery) < 5.0){
			return;
		}

		lastWifiQuery = now;
		wifiHasData = false;
		wifiConnected = false;
		wifiState = "unknown";
		wifiSsid.clear();

		// Read operstate from sysfs
		std::ifstream opf("/sys/class/net/wlan0/operstate");
		if(opf.good()){
			std::getline(opf, wifiState);
			opf.close();
		}

		// Check presence in /proc/net/wireless to guess link availability
		std::ifstream wf("/proc/net/wireless");
		if(wf.good()){
			std::string line;
			while(std::getline(wf, line)){
				if(line.find("wlan0:") != std::string::npos){
					wifiHasData = true;
					// if there is a line, assume interface is up-ish
					wifiConnected = (wifiState == "up" || wifiState == "unknown" || wifiState == "dormant");
					break;
				}
			}
			wf.close();
		}

		// Try to get SSID via iwgetid (does not rely on nmcli)
		FILE* pipe = popen("iwgetid -r 2>/dev/null", "r");
		if(pipe){
			char buf[256] = {0};
			if(fgets(buf, sizeof(buf), pipe)){
				wifiSsid = buf;
				wifiSsid.erase(std::remove(wifiSsid.begin(), wifiSsid.end(), '\n'), wifiSsid.end());
			}
			pclose(pipe);
		}

		if(!wifiHasData && !wifiState.empty()){
			// Still publish something even if /proc/net/wireless missing
			wifiHasData = true;
		}
	}

	void convertState2json(const state_controller_msg::State & state, std::string & json) {
		rapidjson::Document document(rapidjson::kObjectType);
		rapidjson::Document telemetry(rapidjson::kObjectType);

		if( !state.position.header.seq  &&  state.position.status.status < 0) {
			//If we don't have a fix, send an empty position array
			rapidjson::Value positionArray(rapidjson::Type::kArrayType);
			telemetry.AddMember("position", positionArray, telemetry.GetAllocator()); // empty position array
			rapidjson::Value gnssStatus((int)state.position.status.status);
			telemetry.AddMember("gnssFix", gnssStatus, telemetry.GetAllocator());
			
			
		}
	else {
			// Else, send the whole position
			rapidjson::Value positionArray(rapidjson::Type::kArrayType);
			rapidjson::Value longitude(state.position.longitude);
			rapidjson::Value latitude(state.position.latitude);

			positionArray.PushBack(longitude, telemetry.GetAllocator());
			positionArray.PushBack(latitude, telemetry.GetAllocator());
			telemetry.AddMember("position", positionArray, telemetry.GetAllocator());
			rapidjson::Value gnssStatus((int)state.position.status.status);
			telemetry.AddMember("gnssFix", gnssStatus, telemetry.GetAllocator());
		}

		if(!state.imu.header.seq){
			rapidjson::Value attitudeArray(rapidjson::Type::kArrayType);
			telemetry.AddMember("attitude", attitudeArray, telemetry.GetAllocator()); // empty attitude array
		}
		else {
			try{
				double heading = 0;
				double pitch = 0;
				double roll = 0;

				geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));

				QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation,state.imu.orientation,heading,pitch,roll);

		//hydrographers prefer heading from 0-360 in NED frame
		heading = 90-heading;

				if(heading < 0) {
					heading += 360.0;
				}

				rapidjson::Value attitudeArray(rapidjson::Type::kArrayType);
				rapidjson::Value headingValue(heading);
				rapidjson::Value pitchValue(pitch);
				rapidjson::Value rollValue(roll);

				attitudeArray.PushBack(headingValue, telemetry.GetAllocator());
				attitudeArray.PushBack(pitchValue, telemetry.GetAllocator());
				attitudeArray.PushBack(rollValue, telemetry.GetAllocator());
				telemetry.AddMember("attitude", attitudeArray, telemetry.GetAllocator());
			} catch (tf2::TransformException &ex) {
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

		if(!state.vitals.header.seq) {
			rapidjson::Value vitalsArray(rapidjson::Type::kArrayType);
			telemetry.AddMember("vitals", vitalsArray, telemetry.GetAllocator()); // empty vitals array
		}
	else{
			rapidjson::Value vitalsArray(rapidjson::Type::kArrayType);
			rapidjson::Value cputemp((int)state.vitals.cputemp);
			rapidjson::Value cpuload((int) state.vitals.cpuload);
			rapidjson::Value freeram((int) state.vitals.freeram);
			rapidjson::Value freehdd((int) state.vitals.freehdd);
			rapidjson::Value uptime((int) state.vitals.uptime);
			rapidjson::Value temp((int) state.vitals.temperature);
			rapidjson::Value voltage(state.vitals.voltage);
			rapidjson::Value humidity((int) state.vitals.humidity);
			rapidjson::Value ledstate( state.vitals.ledstate);
			rapidjson::Value status;
			status.SetString(state.vitals.status.c_str(), telemetry.GetAllocator());

			vitalsArray.PushBack(cputemp, telemetry.GetAllocator()); //0
			vitalsArray.PushBack(cpuload, telemetry.GetAllocator());
			vitalsArray.PushBack(freeram, telemetry.GetAllocator());
			vitalsArray.PushBack(freehdd, telemetry.GetAllocator());
			vitalsArray.PushBack(uptime, telemetry.GetAllocator()); //4
			vitalsArray.PushBack(temp, telemetry.GetAllocator()); //5
			vitalsArray.PushBack(voltage, telemetry.GetAllocator());//6
			vitalsArray.PushBack(humidity, telemetry.GetAllocator());//7
			vitalsArray.PushBack(ledstate, telemetry.GetAllocator());

			telemetry.AddMember("vitals", vitalsArray, telemetry.GetAllocator());
			telemetry.AddMember("status", status, telemetry.GetAllocator());
		}

		// Wi-Fi info
		refreshWifiInfo();
		rapidjson::Value wifi(rapidjson::Type::kObjectType);
		const std::string ifaceName = "wlan0";
		rapidjson::Value iface;
		iface.SetString(ifaceName.c_str(), static_cast<rapidjson::SizeType>(ifaceName.size()), telemetry.GetAllocator());
		wifi.AddMember("interface", iface, telemetry.GetAllocator());

		rapidjson::Value stateStr;
		stateStr.SetString(wifiState.c_str(), static_cast<rapidjson::SizeType>(wifiState.size()), telemetry.GetAllocator());
		wifi.AddMember("state", stateStr, telemetry.GetAllocator());
		wifi.AddMember("connected", wifiConnected, telemetry.GetAllocator());

		rapidjson::Value ssidVal;
		ssidVal.SetString(wifiSsid.c_str(), static_cast<rapidjson::SizeType>(wifiSsid.size()), telemetry.GetAllocator());
		wifi.AddMember("ssid", ssidVal, telemetry.GetAllocator());
		wifi.AddMember("hasData", wifiHasData, telemetry.GetAllocator());

		telemetry.AddMember("wifi", wifi, telemetry.GetAllocator());

		document.AddMember("telemetry", telemetry, document.GetAllocator());
		rapidjson::StringBuffer sb;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
		document.Accept(writer);
		json = sb.GetString();

	}

	void stateChanged(const state_controller_msg::State & state) {
		uint64_t timestamp = (state.stamp.sec * 1000000) + (state.stamp.nsec/1000);
		//TODO: maybe add our own header?
		if(timestamp - lastTimestamp > 200000){
			std::string jsonString;
			convertState2json(state, jsonString);
			
			bool isLogging = getRecordingStatus();
			int loggingMode = getLoggingMode();
			
			if(loggingMode == 1 && !isLogging){
				logger_service::ToggleLogging toggle;
				toggle.request.loggingEnabled = true;
				toggleLoggingService.call(toggle);
				isLogging = getRecordingStatus();
				loggingMode = getLoggingMode();
			}
			std::lock_guard<std::mutex> lock(mtx);
			for (auto it : connections) {
				 srv.send(it,jsonString,websocketpp::frame::opcode::text);
				 sendRecordingInfo(it,isLogging, loggingMode);
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

		ROS_INFO("Stopping Telemetry server");
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
	ros::ServiceClient getLoggingModeServiceClient;
	ros::ServiceClient setLoggingModeServiceClient;
	ros::Subscriber gnssStatusSub;

	// Wi-Fi cache
	double lastWifiQuery = 0.0;
	bool wifiHasData = false;
	bool wifiConnected = false;
	std::string wifiState = "unknown";
	std::string wifiSsid;

	
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener transformListener;

	uint64_t lastTimestamp;
};

#endif
