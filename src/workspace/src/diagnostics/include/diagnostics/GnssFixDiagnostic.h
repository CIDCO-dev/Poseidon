#ifndef GNSSFIXDIAGNOISTIC_H
#define GNSSFIXDIAGNOISTIC_H

#include "DiagnosticsTest.h"
#include "sensor_msgs/NavSatFix.h"

class GnssFixDiagnostic : public DiagnosticsTest{
	
public:
	GnssFixDiagnostic(std::string name, int _messageFrequency): DiagnosticsTest(name), messageFrequency(_messageFrequency){}
	~GnssFixDiagnostic(){}
	
	
	void callback(const sensor_msgs::NavSatFix& gnss){
		messageCount++;
		if(gnss.status.service >= 0){
			fix++;
		}
	}
	
	void do_test()override{
		try{
			//reinitialize test results
			this->status = false;
			this->value = "";
			this->fix = 0;
			messageCount = 0;
			
			subscriber = node.subscribe("fix", 10, &GnssFixDiagnostic::callback, this);
			
			ros::Time startTime = ros::Time::now();
			double elapsedTime = 0.0;
			//loop while messageCount condition is not met or exit loop if timer has expired
			while (ros::ok()){
				
				ros::Time currentTime = ros::Time::now();
				elapsedTime = (currentTime - startTime).toSec();
			
				if (fix >= (messageFrequency * sleepDuration)){
					break;
				}
				
				if (elapsedTime >= sleepDuration){
					break;
				}
				ros::Duration(0.1).sleep();
			}
			
			double fixMsgPourcent = ((double)fix / (double)messageCount) * 100;
			
			if(fixMsgPourcent > 90){
				this->value += std::to_string(fixMsgPourcent) + "% " + "of messages has fix \n";
				this->status = true;
			}
			else{
				this->value += "No gnss fix \n";
				this->status = false;
			}
			subscriber.shutdown();
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
	int messageFrequency;
	int fix = 0;
	int messageCount = 0;

};
#endif
