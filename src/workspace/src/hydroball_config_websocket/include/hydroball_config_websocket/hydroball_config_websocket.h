#ifndef hydroball_config_websocket
#define hydroball_config_websocket

#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <map>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include <ros/console.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>


#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "setting_msg/ImuOffsetService.h"

#include "state_controller_msg/GetStateService.h"

#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"


typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;

class ConfigurationServer {
public:
	ConfigurationServer(std::string & configFilePath): configFilePath(configFilePath) {
		srv.init_asio();
		srv.set_reuse_addr(true);
		srv.clear_access_channels(websocketpp::log::alevel::all); 
		srv.set_open_handler(bind(&ConfigurationServer::on_open,this,std::placeholders::_1));
		srv.set_close_handler(bind(&ConfigurationServer::on_close,this,std::placeholders::_1));
		srv.set_message_handler(bind(&ConfigurationServer::on_message,this,std::placeholders::_1,std::placeholders::_2));

		configTopic     = n.advertise<setting_msg::Setting>("configuration", 1000);
		configService   = n.advertiseService("get_configuration", &ConfigurationServer::getConfigurationService,this);
		zeroImuService  = n.advertiseService("zero_imu_offsets", &ConfigurationServer::zeroImuOffsetService,this);
		getStateClient  = n.serviceClient<state_controller_msg::GetStateService>("get_state");

		readConfigurationFromFile();
		broadcastImuTransform();
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

			if(command.compare("getConfiguration")==0){
				sendConfiguration(hdl);
			}
			else if(command.compare("saveConfiguration")==0){
				saveConfiguration(document);
			}
			else if(command.compare("zeroImu")==0){
				setting_msg::ImuOffsetService::Request  request;
				setting_msg::ImuOffsetService::Response response;
				zeroImuOffsetService(request,response);
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

	//Read configuration from file
	void readConfigurationFromFile(){
		std::ifstream in;

		in.open(configFilePath);

		if(in.is_open()){
			std::string line;
			while(std::getline(in,line)){
				std::stringstream ss(line);
				std::string key;
				std::string value;

				ss >> key >> value;

				if(key.size() > 0 && value.size() > 0){
					configuration[key]=value;
				}
			}

			in.close();
		}
		else{
			throw std::invalid_argument(std::string("Cannot open file ") + configFilePath);
		}
	}

	//Write configuration to file
	void writeConfigurationToFile(){
                std::ofstream out;

                out.open(configFilePath,std::ofstream::trunc);

                if(out.is_open()){
                        for(auto i = configuration.begin();i!=configuration.end();i++){
				out << i->first << " " << i->second << std::endl;
                        }

                        out.close();
                }
                else{
                        throw std::invalid_argument(std::string("Cannot open file ") + configFilePath);
                }

	}

	//Send settings as a JSON object
	void sendConfiguration(connection_hdl & hdl){
		rapidjson::Document document;
		document.SetObject();

		rapidjson::Value confArray(rapidjson::Type::kArrayType);

		for(auto i = configuration.begin();i!=configuration.end();i++){
			rapidjson::Value setting(rapidjson::Type::kObjectType);

			rapidjson::Value key;
			key = rapidjson::StringRef(i->first.c_str());

			rapidjson::Value value;
			value = rapidjson::StringRef(i->second.c_str());

			setting.AddMember("key",key,document.GetAllocator());
			setting.AddMember("value",value,document.GetAllocator());
			confArray.PushBack(setting,document.GetAllocator());
		}

		document.AddMember("configuration",confArray,document.GetAllocator());

		rapidjson::StringBuffer sb;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
		document.Accept(writer);
		std::string jsonString = sb.GetString();

		srv.send(hdl,jsonString,websocketpp::frame::opcode::text);
	}

	void saveConfiguration(rapidjson::Document & document){

		if(document.HasMember("configuration") && document["configuration"].IsArray()){
			for (rapidjson::SizeType i = 0; i < document["configuration"].Size(); i++){
				if(document["configuration"][i].HasMember("key") && document["configuration"][i].HasMember("value")){
					std::string key = document["configuration"][i]["key"].GetString();
					std::string value = document["configuration"][i]["value"].GetString();

					if(configuration.find(key) != configuration.end()){
						configuration[key]=value;
					}
					else{
						std::cout << "Key not found" << std::endl;
					}
				}
				else{
					std::cout << "Configuration objet without key/value" << std::endl;
				}
			}

			writeConfigurationToFile();
			broadcastConfiguration();
			broadcastImuTransform();
		}
		else{
			ROS_ERROR("%s","Configuration array not found");
		}
	}

	//Broadcasts configuration changes on the configuration topic
	void broadcastConfiguration(){
		for(auto i=configuration.begin();i!=configuration.end();i++){
			setting_msg::Setting setting;
			setting.key   =i->first;
			setting.value =i->second;

			configTopic.publish(setting);
		}
	}

	void broadcastImuTransform(){
		if(
			configuration.find("headingOffset")!=configuration.end() &&
			configuration.find("pitchOffset")!=configuration.end() &&
			configuration.find("rollOffset")!=configuration.end()
		){
			double headingOffset;
			double pitchOffset;
			double rollOffset;

			if(
				sscanf(configuration["headingOffset"].c_str(),"%lf",&headingOffset)==1 &&
				sscanf(configuration["pitchOffset"].c_str()  ,"%lf",&pitchOffset  )==1 &&
				sscanf(configuration["rollOffset"].c_str()   ,"%lf",&rollOffset   )==1
			){
				//init broadcaster
				static tf2_ros::StaticTransformBroadcaster broadcaster;

				//init transform quaternion
				tf2::Quaternion q;
				q.setRPY(D2R(rollOffset),D2R(pitchOffset),D2R(headingOffset));

				//pack transform and send
				geometry_msgs::TransformStamped imuTransform;

				imuTransform.header.stamp         = ros::Time::now();
				imuTransform.header.frame_id      = "base_link";
				imuTransform.child_frame_id       = "imu";
				imuTransform.transform.rotation.x = q.x();
				imuTransform.transform.rotation.y = q.y();
				imuTransform.transform.rotation.z = q.z();
				imuTransform.transform.rotation.w = q.w();

				broadcaster.sendTransform(imuTransform);
			}
		}
	}

	bool getConfigurationService(setting_msg::ConfigurationService::Request & req,setting_msg::ConfigurationService::Response & res){
                if(configuration.find(req.key) != configuration.end()){
	                res.value = configuration[req.key];
                }

		return true;
	}

	bool zeroImuOffsetService(setting_msg::ImuOffsetService::Request & req, setting_msg::ImuOffsetService::Response & res){

		state_controller_msg::GetStateService srv;

		if(getStateClient.call(srv)){
			double headingOffset;
			double pitchOffset;
			double rollOffset;

			tf2::Quaternion q;
			tf2::fromMsg(srv.response.state.imu.orientation,q);
			tf2::Matrix3x3 mat(q);
			mat.getEulerYPR(headingOffset,pitchOffset,rollOffset);

			//Offsets are negated so that steady-state angle + offset = 0
			configuration["headingOffset"] = std::to_string(-R2D(headingOffset));
			configuration["pitchOffset"]   = std::to_string(-R2D(pitchOffset));
			configuration["rollOffset"]    = std::to_string(-R2D(rollOffset));

                        writeConfigurationToFile();
                        broadcastConfiguration();
			broadcastImuTransform();

			return true;
		}

		return false;
	}

private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server 		srv;
    con_list 		connections;
    std::mutex 		mtx;

    ros::NodeHandle 	n;
    ros::Publisher	configTopic;
    ros::ServiceServer	configService;
    ros::ServiceServer  zeroImuService;
    ros::ServiceClient  getStateClient;


    std::string 	configFilePath;

    std::map<std::string,std::string>  configuration;
};

#endif
