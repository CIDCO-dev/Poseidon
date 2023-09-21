#ifndef SONARCOMMUNICATIONDIAGNOISTIC_H
#define SONARCOMMUNICATIONDIAGNOISTIC_H

#include "DiagnosticsTest.h"
#include "geometry_msgs/PointStamped.h"

class SonarCommunicationDiagnostic : public DiagnosticsTest{
	
public:
	SonarCommunicationDiagnostic(std::string name, int _messageFrequency): DiagnosticsTest(name), messageFrequency(_messageFrequency){}
	~SonarCommunicationDiagnostic(){}
	
	void callback(const geometry_msgs::PointStamped& sonar){
		messageCount++;
	}
	
	void do_test()override{
		try{
			//reinitialize test results
			this->status = false;
			this->value = "";
			this->messageCount = 0;
			
			subscriber = node.subscribe("depth", 10, &SonarCommunicationDiagnostic::callback, this);
			
			ros::Time startTime = ros::Time::now();
			double elapsedTime = 0.0;
			//loop while messageCount condition is not met or exit loop if timer has expired
			while (ros::ok()){
				
				ros::Time currentTime = ros::Time::now();
				elapsedTime = (currentTime - startTime).toSec();
			
				if (messageCount >= (messageFrequency * sleepDuration)){
					break;
				}
				
				if (elapsedTime >= sleepDuration){
					break;
				}
				ros::Duration(0.1).sleep();
			}
			
			if(messageCount > 0){
				this->status = true;
				this->value += std::to_string(messageCount) + " message received in " + std::to_string(elapsedTime) + " seconds \n";
			}
			else{
				this->status = false;
				this->value = "No message received in " + std::to_string(elapsedTime) + " seconds";
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
	double sleepDuration = 1.0;
	int messageCount = 0;
	int messageFrequency;
};
#endif
