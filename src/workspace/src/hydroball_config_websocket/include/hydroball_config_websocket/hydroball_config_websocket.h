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
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>
#include <string>

#include "ros/ros.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "setting_msg/Setting.h"

#include "setting_msg/ConfigurationService.h"

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;

class ConfigurationServer {
public:
	ConfigurationServer(std::string & configFilePath): configFilePath(configFilePath) {
		srv.init_asio();
		srv.set_reuse_addr(true);
		srv.set_open_handler(bind(&ConfigurationServer::on_open,this,std::placeholders::_1));
		srv.set_close_handler(bind(&ConfigurationServer::on_close,this,std::placeholders::_1));
		srv.set_message_handler(bind(&ConfigurationServer::on_message,this,std::placeholders::_1,std::placeholders::_2));

		configTopic     = n.advertise<setting_msg::Setting>("configuration", 1000);
		configService   = n.advertiseService("get_configuration", &ConfigurationServer::getConfigurationService,this);

		readConfigurationFromFile();
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

	void receiveMessages(){
		ros::spin();
	}

	void run(uint16_t port){
		srv.listen(port);
		srv.start_accept();
		srv.run();
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

	bool getConfigurationService(setting_msg::ConfigurationService::Request & req,setting_msg::ConfigurationService::Response & res){
                if(configuration.find(req.key) != configuration.end()){
	                res.value = configuration[req.key];
                }

		return true;
	}
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server 		srv;
    con_list 		connections;
    std::mutex 		mtx;

    ros::NodeHandle 	n;
    ros::Publisher	configTopic;
    ros::ServiceServer	configService;
    std::string 	configFilePath;

    std::map<std::string,std::string>  configuration;
};

#endif
