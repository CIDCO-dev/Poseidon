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

class WifiConfig{
	
	private:
		ros::NodeHandle node;
		ros::Subscriber configurationSubscriber;
		ros::ServiceClient	configurationClient;
		
		bool wifiTransferActivated = false;
		std::string ssid;
		std::string wifiPassword;
		
		std::string newWifiPassword = "";
		std::string newSsid = "";
	
	
	public:
		WifiConfig(){
			configurationSubscriber = node.subscribe("configuration", 1000, &WifiConfig::configurationCallBack, this);
			configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
			get_wifi_config();
			if(wifiTransferActivated && !is_connection_set()){
				set_wifi_config();
			}
		};
		
		~WifiConfig(){};
		
		void get_wifi_config(){
			
			setting_msg::ConfigurationService srv;
			
			srv.request.key = "ssid";
			if(configurationClient.call(srv)){
				this->ssid = trimSpaces(srv.response.value);
			}
			
			srv.request.key = "wifiPassword";
			if(configurationClient.call(srv)){
				this->wifiPassword = srv.response.value;
			}
			
			if(this->ssid.size() > 0){
				this->wifiTransferActivated = true;
			}
		}
		
		void set_wifi_config(){
			
			std::string command = "nmcli connection add type wifi ssid " + this->ssid;	
			command += 	" wifi-sec.key-mgmt \"wpa-psk\" wifi-sec.psk " + this->wifiPassword;
			command += " autoconnect yes con-name " + this->ssid;
			
			if( std::system(command.c_str()) != 0){
				ROS_ERROR_STREAM("Could not create connection");
			}
		}
		
		// Callback for when configs are changed by the user via the web ui
		void configurationCallBack(const setting_msg::Setting &setting){
			ROS_INFO_STREAM("logger_text configCallback -> " << setting.key << " : "<<setting.value<<"\n");
			if(setting.key == "ssid"){
				std::string temp = setting.value;
				this->newSsid = trimSpaces(temp);
			}
			
			else if(setting.key == "wifiPassword"){
				this->newWifiPassword = setting.value;
			}
			
			// new connection configs
			if(newSsid.size() > 0 && newWifiPassword.size() > 0 && 
				this->ssid != this->newSsid && this->wifiPassword != this->newWifiPassword){
				
				deleteOldConnection();
				this->ssid = this->newSsid;
				this->wifiPassword = this->newWifiPassword;
				
				this->newSsid = "";
				this->newWifiPassword = "";
				
				set_wifi_config();
				
			}
			// disabling transfer
			else if(newSsid.size() == 0 && this->ssid != this->newSsid && this->wifiPassword != this->newWifiPassword){
			
				deleteOldConnection();
				this->wifiTransferActivated = false;
			}
			
		}
		
		bool is_connection_set(){
			
			std::string command = "nmcli -f GENERAL.STATE con show " + this->ssid;
			if(std::system(command.c_str()) != 0){
				return false;
			}
			else{
				return true;
			}
		}
		
		void deleteOldConnection(){
			std::string command = "nmcli con delete " + this->ssid;
			if(std::system(command.c_str()) != 0){
				ROS_ERROR("could not delete old connection");
			}
		}
		
};

#endif
