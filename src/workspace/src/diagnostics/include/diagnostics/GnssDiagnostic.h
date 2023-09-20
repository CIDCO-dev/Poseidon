#ifndef GNSSDIAGNOISTIC_H
#define GNSSDIAGNOISTIC_H

#include "DiagnosticsTest.h"

class GnssDiagnostic : public DiagnosticsTest{
	
public:
	GnssDiagnostic(std::string name, std::string topic, int messageFrequency): DiagnosticsTest(name, topic, messageFrequency){}
	~GnssDiagnostic(){}
	
	
	void messageCallback(const sensor_msgs::NavSatFix& gnss){
		messageCount++;
		if(gnss.status.service >= 0){
			fix++;
		}
	}
	
	void start_subscriber() override{
		//reinitialize test results
		this->status = false;
		this->value = "";
		messageCount = 0;
		fix = 0;
		
		subscriber = node.subscribe(this->topic, 10, &GnssDiagnostic::messageCallback, this);
	}
	
	bool has_gnss_fix(){
		
		if(fix > 0){
			this->value += std::to_string((((double)fix/(double)messageCount))*100) + "% " + "of messages has fix \n";
			return true;
		}
		else{
			this->value += "No gnss fix \n";
			return false;
		}
		
	}
	
	void do_tests()override{
		
		if(is_receiving_messages() && has_gnss_fix()){ //add other test here
			this->status = true;
		}
		else{
			this->status = false;
		}
	}
	
	
private:
	int fix = 0;

};
#endif
