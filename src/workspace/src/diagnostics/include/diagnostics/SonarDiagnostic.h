#ifndef SONARDIAGNOISTIC_H
#define SONARDIAGNOISTIC_H

#include "DiagnosticsTest.h"

class SonarDiagnostic : public DiagnosticsTest{
	
public:
	SonarDiagnostic(std::string name, std::string topic, int messageFrequency): DiagnosticsTest(name, topic, messageFrequency){}
	~SonarDiagnostic(){}
	
	void messageCallback(const geometry_msgs::PointStamped& sonar){
		messageCount++;
	}
	
	void start_subscriber() override{
		//reinitialize test results
		this->status = false;
		this->value = "";
		messageCount = 0;
		
		subscriber = node.subscribe(this->topic, 10, &SonarDiagnostic::messageCallback, this);
	}
	
	void gnss_fix(){
		
	}
	
	void do_tests()override{
		
		if(is_receiving_messages() ){ //add other test here
			this->status = true;
		}
		
	}
	
	
private:

};
#endif
