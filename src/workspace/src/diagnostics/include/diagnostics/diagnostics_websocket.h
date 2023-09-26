#include <mutex>
#include <set>
#include <thread>
#include <iostream>
#include <vector>

#include <boost/assign/list_of.hpp>

#include "ros/ros.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "DiagnosticsTest.h"
#include "GnssCommunicationDiagnostic.h"
#include "GnssFixDiagnostic.h"
#include "ImuCommunicationDiagnostic.h"
#include "ImuCalibrationDiagnostic.h"
#include "SonarCommunicationDiagnostic.h"

typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;

class DiagnosticsServer {
public:
	DiagnosticsServer(){
		srv.init_asio();
		srv.set_reuse_addr(true);
		srv.clear_access_channels(websocketpp::log::alevel::all);

		srv.set_open_handler(bind(&DiagnosticsServer::on_open, this, std::placeholders::_1));
		srv.set_close_handler(bind(&DiagnosticsServer::on_close, this, std::placeholders::_1));
		srv.set_message_handler(bind(&DiagnosticsServer::on_message, this, std::placeholders::_1, std::placeholders::_2));
		
		
		diagnosticsVector = boost::assign::list_of<DiagnosticsTest*>(new GnssCommunicationDiagnostic("Gnss Communication", 1))
																	(new GnssFixDiagnostic("Gnss fix", 1))
																	(new ImuCommunicationDiagnostic("Imu Communication", 100))
																	(new ImuCalibrationDiagnostic("Imu Calibrated", 100))
																	(new SonarCommunicationDiagnostic("SonarDiagnostic", 10))
																	;
	}
	
	~DiagnosticsServer(){
		for(auto obj = diagnosticsVector.begin(); obj != diagnosticsVector.end(); obj++){
			delete *obj;
		}
	}
	
	void build_running_nodes_array(rapidjson::Document &document){
		std::vector<std::string> runningNodes;
		bool state = ros::master::getNodes(runningNodes);
		
		if(!state){
			ROS_ERROR("DiagnosticsServer::build_running_nodes_array() error with ros::master::getNodes()");
		}
		else{
			//document.SetObject();
			rapidjson::Value runningNodesArray(rapidjson::kArrayType);

			for(auto &node: runningNodes){
				rapidjson::Value strValue;
				strValue.SetString(node.c_str(), node.length(), document.GetAllocator());
				runningNodesArray.PushBack(strValue, document.GetAllocator());
			}
			document.AddMember("running_nodes", runningNodesArray, document.GetAllocator());
		}
	}
	
	void send_json(rapidjson::Document &document){
	
		rapidjson::StringBuffer sb;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
		document.Accept(writer);
		
		std::lock_guard<std::mutex> lock(mtx);
		for (auto it : connections) {
			 srv.send(it, sb.GetString(), websocketpp::frame::opcode::text);
		}
	}
	
	void build_diagnostics(rapidjson::Document &document){
		
		rapidjson::Value diagnosticsArray(rapidjson::kArrayType);
		
		for(auto obj: diagnosticsVector){
			obj->do_test();
			obj->to_json(document, diagnosticsArray);
		}
		document.AddMember("diagnostics", diagnosticsArray, document.GetAllocator());
	}
	
	void on_message(connection_hdl hdl, server::message_ptr msg) {
	
		rapidjson::Document document;

		if(document.Parse(msg->get_payload().c_str()).HasParseError()){
			//Not valid JSON
			rapidjson::StringBuffer sb;
			rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
			document.Accept(writer);
			std::string jsonString = sb.GetString();
			ROS_ERROR_STREAM("Error parsing JSON on_message: " << jsonString);
			return;
		}
		
		if(!(document.HasMember("command") && document["command"].IsString())) {
			//no command found. ignore
			ROS_ERROR_STREAM("No command found : " << msg->get_payload());
		}
		else {
			std::string command = document["command"].GetString();
			if(command == "updateDiagnostic"){
				//std::cout<<"updateDiagnostic \n";
				document.SetObject();
				build_diagnostics(document);
				send_json(document);
			}
			else if(command == "getRunningNodes"){
				//std::cout<<"getRunningNodes \n";
				document.SetObject();
				build_running_nodes_array(document);
				send_json(document);
			}
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
		std::lock_guard<std::mutex> lock(mtx);
		for (auto it : connections) {
				websocketpp::lib::error_code ec_close_connection;
				srv.close(it,websocketpp::close::status::normal,closingMessage, ec_close_connection);
				if(ec_close_connection) {
					ROS_ERROR_STREAM("failed to close connection: " << ec_close_connection.message());
			 	}
			}

		ROS_INFO("Stopping Diagnostic server");
			srv.stop();
	 }

private:
	typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
	server srv;
	con_list connections;
	std::mutex mtx;
	std::vector<DiagnosticsTest*> diagnosticsVector;
};
