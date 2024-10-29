
#ifndef wifi_config
#define wifi_config

//Ros
#include "ros/ros.h"

//Poseidon
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "../../utils/string_utils.hpp"

//C++ std lib
#include <iostream>
#include <filesystem>
#include <sstream>
#include <fstream>


/*
	you cannot delete a connection as it would require a seperate webpage to manage connection individually
	instead of deleting a connection, disable it and/or change password
*/

class WifiConfig{
	
	private:
		ros::NodeHandle node;
		ros::Subscriber configurationSubscriber;
		ros::ServiceClient	configurationClient;
		std::map<std::string, std::map<std::string, std::string>> wifiConnections;
		
		std::string currentSsid;
		std::string currentPassword;
		bool autoconnectStatus;
		
		unsigned int configCallbackCounter = 0;
		
		void print_wifi_connections(){
			
			for(auto connection: wifiConnections){
				auto config = wifiConnections[connection.first];
				
				std::cout<<"Con: " << connection.first << " " << config["autoconnect"] << " " << config["password"] << " \n";
			}
			
		}
		
		void get_auto_connect_status(){
			char buffer[256];
			std::string line, ssid, autoconnect;
			bool firstLine = true;
			
			std::shared_ptr<FILE> pipe(popen("nmcli -f NAME,AUTOCONNECT con show", "r"), pclose);
			if (!pipe) {
				throw std::runtime_error("popen() failed!");
			}
			while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
				line = buffer;
				
				// Skip the header line
				if (firstLine) {
					firstLine = false;
					continue;
				}
				
				std::istringstream iss(line);
				
				if (iss >> ssid >> autoconnect) {
					//std::cout << "ssid: " << ssid << ", autoconnect: " << autoconnect << std::endl;
					auto& config = wifiConnections[ssid];
					config["autoconnect"] = autoconnect;
					
				} 
				else {
					std::cerr << "Failed to parse line: " << line << std::endl;
				}
			}
		}
		
		void add_connection_infos(const std::string &path){
			std::ifstream file(path);
			if (!file.is_open()) {
				std::cerr << "Error opening file: " << path << std::endl;
				return;
			}
			
			std::string line;
			std::string ssid, password, id;

			while (std::getline(file, line)) {
				// Trim leading spaces
				line = trimSpaces(line);

				if (line.rfind("ssid=", 0) == 0) {
					ssid = trimSpaces(line.substr(5)); // Extract SSID
				}
				else if (line.rfind("psk=", 0) == 0) {
					password = trimSpaces(line.substr(4)); // Extract password
				}
				else if (line.rfind("id=", 0) == 0) {
					id = trimSpaces(line.substr(3)); // Extract password
				}
			}
			file.close();
			
			if(ssid == id){
				std::map<std::string, std::string> config;
				config["password"] = password;
				wifiConnections[ssid] = config; // XXX watchout for duplicates
			}
			else{
				// TODO
				// connection name should be the same has ssid
			}
			
		}
		
		void get_nmcli_connections(){
			try {
				for (const auto& entry : std::filesystem::directory_iterator("/etc/NetworkManager/system-connections/")) {
					add_connection_infos(entry.path());
				}
			} catch (const std::filesystem::filesystem_error& err) {
				std::cerr << "Error: " << err.what() << std::endl;
			}
			
			get_auto_connect_status();
		}
	
		void add_connection_2_nmcli(const std::string &ssid, const std::string &password, const std::string &autoconnect){
			
			std::string command = "nmcli connection add type wifi ssid " + ssid;
			command += 	" wifi-sec.key-mgmt wpa-psk wifi-sec.psk " + password;
			command += " autoconnect " + autoconnect + " con-name " + ssid;
			
			if( std::system(command.c_str()) != 0){
				ROS_ERROR_STREAM("Could not create connection");
			}
		}
		
		void modify_nmcli_connection(const std::string &ssid, const std::string &password, const std::string &autoconnect){
			
			std::string cmd = "sudo nmcli con modify " + ssid + " wifi-sec.psk ";
			cmd += password;
			cmd += " autoconnect " + autoconnect;
			std::system(cmd.c_str());
			
			cmd = "sudo nmcli con down " + ssid;
			if(std::system(cmd.c_str()) == 0){
				cmd = "sudo nmcli con up " + ssid;
				std::system(cmd.c_str());
			}
		}
		
		void get_wifi_config(){
			
			setting_msg::ConfigurationService srv;
			
			srv.request.key = "wifiSSID";
			if(configurationClient.call(srv)){
				this->currentSsid = srv.response.value;
			}
			
			srv.request.key = "wifiPassword";
			if(configurationClient.call(srv)){
				this->currentPassword = srv.response.value;
			}

			srv.request.key = "wifiTransferEnabled";
			if(configurationClient.call(srv)){
				if(srv.response.value == "true" || srv.response.value == "yes"){
					this->autoconnectStatus = true;
				}
				else{
					this->autoconnectStatus = false;
				}
			}
			
			if (wifiConnections.find(currentSsid) == wifiConnections.end()) {
				
				std::map<std::string, std::string> config;
				config["autoconnect"] = (autoconnectStatus == true) ? "yes" : "no";
				config["password"] = currentPassword;
				wifiConnections[currentSsid] = config;
				
				add_connection_2_nmcli(currentSsid, currentPassword, (autoconnectStatus == true) ? "yes" : "no");
			}
			else{
				modify_nmcli_connection(currentSsid, currentPassword, (autoconnectStatus == true) ? "yes" : "no");
			}
			
		}
	
		// Callback for when configs are changed by the user via the web ui
		void configurationCallBack(const setting_msg::Setting &setting){
			//ROS_INFO_STREAM("wifi_config configCallback -> " << setting.key << " : "<<setting.value<<"\n");
			if(setting.key == "wifiSSID"){
				configCallbackCounter++;
				if(setting.value.size() > 0){
					this->currentSsid = setting.value;
				}
			}
			
			else if(setting.key == "wifiPassword"){
				configCallbackCounter++;
				if(setting.value.size() >= 8){
					this->currentPassword = setting.value;
				}
			}
			
			else if(setting.key == "wifiTransferEnabled"){
				configCallbackCounter++;
				this->autoconnectStatus = (setting.value == "true" || setting.value == "yes") ? true : false;
				}
			
			if(configCallbackCounter >= 3){
				if (wifiConnections.find(currentSsid) == wifiConnections.end()) {
					
					std::map<std::string, std::string> config;
					config.insert({"autoconnect", (autoconnectStatus == true) ? "yes" : "no"});
					config.insert({"password", currentPassword});
					wifiConnections[currentSsid] = config;
					
					add_connection_2_nmcli(currentSsid, currentPassword, (autoconnectStatus == true) ? "yes" : "no");
				}
				else{
					modify_nmcli_connection(currentSsid, currentPassword, (autoconnectStatus == true) ? "yes" : "no");
				}
				
				configCallbackCounter = 0;
			}
		}
	
	public:
		WifiConfig(){
			configurationSubscriber = node.subscribe("configuration", 1000, &WifiConfig::configurationCallBack, this);
			configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
			configurationClient.waitForExistence();
			get_nmcli_connections();
			//print_wifi_connections();
			
			get_wifi_config();
			ROS_INFO_STREAM("Wifi config : wifiTransferEnabled: " << autoconnectStatus  << " ssid: " << this->currentSsid << " password: " << this->currentPassword);
		}
		
		~WifiConfig(){}
		
		
		
		
};




#endif
