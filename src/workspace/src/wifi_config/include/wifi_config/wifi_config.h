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
			configurationClient.waitForExistence();
			get_wifi_config();
			ROS_INFO_STREAM("Wifi config : wifiTransferEnabled: " << wifiTransferActivated  << " ssid: " << this->ssid << " password: " << this->wifiPassword);
			if(wifiTransferActivated){
				deleteOldConnection();
				set_wifi_config();
			}
		};
		
		~WifiConfig(){};
		
		void get_wifi_config(){
			
			setting_msg::ConfigurationService srv;
			
			srv.request.key = "wifiSSID";
			if(configurationClient.call(srv)){
				this->ssid = trimSpaces(srv.response.value);
			}
			
			srv.request.key = "wifiPassword";
			if(configurationClient.call(srv)){
				this->wifiPassword = trimSpaces(srv.response.value);
			}

			srv.request.key = "wifiTransferEnabled";
			if(configurationClient.call(srv)){
				if(trimSpaces(srv.response.value) == "true"){
					this->wifiTransferActivated = true;
				}
				else{
					this->wifiTransferActivated = false;
				}
			} 
			
		}
		
		void set_wifi_config(){
			
			if(this->ssid.size() <= 0 || this->wifiPassword.size() < 8){
				ROS_INFO_STREAM("password must have at least 8 caracter and ssid at least one");
				return;
			}
			
			std::string command = "nmcli connection add type wifi ssid " + this->ssid;	
			command += 	" wifi-sec.key-mgmt wpa-psk wifi-sec.psk " + this->wifiPassword;
			command += " autoconnect yes con-name " + this->ssid;
			
			//ROS_ERROR_STREAM(command);
			
			if( std::system(command.c_str()) != 0){
				ROS_ERROR_STREAM("Could not create connection");
			}
		}
		
		// Callback for when configs are changed by the user via the web ui
		void configurationCallBack(const setting_msg::Setting &setting){
			ROS_INFO_STREAM("wifi_config configCallback -> " << setting.key << " : "<<setting.value<<"\n");
			if(setting.key == "wifiSSID"){
				std::string temp = setting.value;
				if(trimSpaces(temp) != this->ssid){
					this->newSsid = trimSpaces(temp);
				}
			}
			
			else if(setting.key == "wifiPassword"){
				std::string temp = setting.value;
				if(trimSpaces(temp) != this->wifiPassword){
					this->newWifiPassword = trimSpaces(temp);
				}
			}
			
			else if(setting.key == "wifiTransferEnabled"){
				std::string temp = setting.value;
				
				// activate wifi transfer
				if(trimSpaces(temp) == "true" && !wifiTransferActivated){
					this->wifiTransferActivated = true;
					if(!is_connection_set()){
						set_wifi_config();
					}
				}
				// deactivate wifi transfer
				else if(trimSpaces(temp) != "true" && wifiTransferActivated){
					this->wifiTransferActivated = false;
					if(is_connection_set()){
						deleteOldConnection();
					}
				}
				else if( trimSpaces(temp) != "true" && trimSpaces(temp) != "false" ){
					ROS_ERROR("wifiTransferEnabled is not set to true or false");
				}
				else{
					// no state changed
				}
			}
			
			// password changed
			if(wifiTransferActivated && this->wifiPassword != this->newWifiPassword ){ 
			
				if(is_connection_set()){
					deleteOldConnection();
				}
				
				this->wifiPassword = this->newWifiPassword;
				set_wifi_config();
			}
			
			// ssid changed
			else if(wifiTransferActivated && this->ssid != this->newSsid){
			
				if(is_connection_set()){
					deleteOldConnection();
				}
				
				this->ssid = this->newSsid;
				set_wifi_config();
			}
			
			// both changed
			else if(wifiTransferActivated && this->ssid != this->newSsid && this->wifiPassword != this->newWifiPassword
					&& this->newWifiPassword != "" && this->newSsid != ""){
				
				deleteOldConnection();
				
				this->ssid = this->newSsid;
				this->wifiPassword = this->newWifiPassword;
				
				set_wifi_config();
			}
		}
		
		bool is_connection_set(){
			
			if(this->ssid.size() <= 0){
				return false;
			}
			
			std::string command = "nmcli -f GENERAL.STATE con show " + this->ssid;
			if(std::system(command.c_str()) != 0){
				//ROS_ERROR_STREAM(command);
				return false;
			}
			else{
				return true;
			}
		}
		
		void deleteOldConnection(){
			
			if(this->ssid.size() <= 0){
				return;
			}
		
			std::string command = "nmcli con delete " + this->ssid;
			if(std::system(command.c_str()) != 0){
				ROS_ERROR("could not delete old connection");
				ROS_ERROR_STREAM(command);
			}
		}
		
};

#endif
