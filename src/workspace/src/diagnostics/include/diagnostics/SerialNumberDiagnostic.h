#pragma once

#include "DiagnosticsTest.h"
#include <unistd.h>
#include <regex>

class SerialNumberDiagnostic : public DiagnosticsTest{
	
public:
	SerialNumberDiagnostic(std::string name): DiagnosticsTest(name){}
	~SerialNumberDiagnostic(){}
	
	
	void do_test()override{
		try{
			//reinitialize test results
			this->status = false;
			this->value = "";
			
			char hostname[256];
			int status = gethostname(hostname, 256);
			std::string hostnameStr(hostname);
			
			if(status != 0){
				this->value = "Could not get hostname \n";
				return;
			}
			
			std::regex pattern("^[A-Za-z]{2,8}-\\d{6}-\\d{3}$");
			if (std::regex_match(hostnameStr, pattern)) {
				this->status = true;
				this->value = "Valid serial number pattern";
			}
			else{
				this->status = false;
				this->value = "Not valid serial number pattern";
			}
		}
		catch(const std::exception& ex){
			ROS_ERROR_STREAM(ex.what());
			this->status = false;
			this->value = ex.what();
			subscriber.shutdown();
		}
	}
	
	
private:
	ros::Subscriber subscriber;
	ros::NodeHandle node;
	double sleepDuration = 1.5;
	int messageCount = 0;
	int messageFrequency;

};
