#pragma once

#include "DiagnosticsTest.h"
#include <iostream>
#include "setting_msg/ConfigurationService.h"
#include "../../utils/HttpClient.hpp"
#include "../../utils/string_utils.hpp"


class ApiConnectionDiagnostic : public DiagnosticsTest{
	
public:
	ApiConnectionDiagnostic(std::string name): DiagnosticsTest(name){
		configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
	}
	~ApiConnectionDiagnostic(){}

	void get_config(){
		
		setting_msg::ConfigurationService srv;

		srv.request.key = "apiServer";

		if(configurationClient.call(srv)){
			try{
				this->host = trimSpaces(srv.response.value);
				
			}
			catch(const std::exception& ex){
				ROS_ERROR_STREAM(ex.what());
				ROS_ERROR("Error in server target definition");
				this->host = "";
			}
		}
		else{
			ROS_WARN("No server target definition");
			this->host = "";
		}
		
		srv.request.key = "apiPort";

		if(configurationClient.call(srv)){
			try{
				this->port = trimSpaces(srv.response.value);
			}
			catch(const std::exception& ex){
				ROS_ERROR_STREAM(ex.what());
				ROS_ERROR("Error in server port definition");
				this->port = "8080";
			}
		}
		else{
			ROS_WARN("No server port definition");
			this->port = "8080";
		}
		
/*		srv.request.key = "apiKey";*/
/*		if(configurationClient.call(srv)){*/
/*			try{*/
/*				this->apiKey = trimSpaces(srv.response.value);*/
/*			}*/
/*			catch(const std::exception& ex){*/
/*				ROS_ERROR_STREAM(ex.what());*/
/*				ROS_ERROR("Error in API key definition");*/
/*				this->apiKey = "";*/
/*			}*/
/*		}*/
/*		else{*/
/*			ROS_WARN("No API key definition");*/
/*			this->apiKey = "";*/
/*		}*/
}


	void do_test()override{
		
		this->status = false;
		this->value = "";
		get_config();
		
		this->status = HttpClient::can_reach_server(this->host, this->port);
		//this->status = HttpsClient::can_reach_server(this->host, this->port);
	}

private:
	ros::NodeHandle node;
	ros::ServiceClient configurationClient;
	
	std::string host;
	std::string port;
	

};
