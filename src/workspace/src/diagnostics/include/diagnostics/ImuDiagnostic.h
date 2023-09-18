#ifndef IMuDIAGNOISTIC_H
#define IMUDIAGNOISTIC_H

#include "DiagnosticsTest.h"
#include <ros/message_traits.h>

class ImuDiagnostic : public DiagnosticsTest{
	
public:
	ImuDiagnostic(std::string name, std::string topic, int messageFrequency): DiagnosticsTest(name, topic, messageFrequency){}
	~ImuDiagnostic(){}
	/*
	void ImuCallback(const sensor_msgs::Imu& imu){
		mutex.lock();
			receivedMessages.push_back(imu);
		mutex.unlock();
	}
	*/
	void messageCallback(const sensor_msgs::Imu& imu){
		messageCount++;
	}
	
	void start_subscriber() override{
		//reinitialize test results
		this->status = false;
		this->value = "";
		//receivedMessages.clear();
		messageCount = 0;
		
		subscriber = node.subscribe(this->topic, 10, &ImuDiagnostic::messageCallback, this);
	}
	
	void do_tests()override{
		start_subscriber();
		
		if(receiving_messages() ){ //add other test here
			this->status = true;
		}
	}
	
private:
	//std::vector<sensor_msgs::Imu> receivedMessages;

};
#endif
