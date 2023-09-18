#ifndef GNSSDIAGNOISTIC_H
#define GNSSDIAGNOISTIC_H

#include "DiagnosticsTest.h"

class GnssDiagnostic : public DiagnosticsTest{
	
public:
	GnssDiagnostic(std::string name, std::string topic, int messageFrequency): DiagnosticsTest(name, topic, messageFrequency){}
	~GnssDiagnostic(){}
	
	/*
	void gnssCallback(const sensor_msgs::NavSatFix& gnss){
		mutex.lock();
			receivedMessage.push_back(gnss);
		mutex.unlock();
	}
	*/
	
	void messageCallback(const sensor_msgs::NavSatFix& gnss){
		messageCount++;
	}
	
	void start_subscriber() override{
		//reinitialize test results
		this->status = false;
		this->value = "";
		//receivedMessages.clear();
		messageCount = 0;
		
		subscriber = node.subscribe(this->topic, 10, &GnssDiagnostic::messageCallback, this);
	}
	
	bool has_gnss_fix(){
		
	}
	
	void do_tests()override{
		
		if(is_receiving_messages() ){ //add other test here
			this->status = true;
		}
	}
	
	
private:
	//std::vector<sensor_msgs::NavSatFix> receivedMessage;

};
#endif
