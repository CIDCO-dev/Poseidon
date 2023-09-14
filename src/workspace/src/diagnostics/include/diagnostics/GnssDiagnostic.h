#ifndef GNSSDIAGNOISTIC_H
#define GNSSDIAGNOISTIC_H

#include "DiagnosticsTest.h"

class GnssDiagnostic : public DiagnosticsTest{
	
public:
	GnssDiagnostic(){
		this->key = "GnssDiagnostic";
	}
	~GnssDiagnostic(){}
	
	void gnssCallback(const sensor_msgs::NavSatFix& gnss){
		mutex.lock();
			receivedMessage.push_back(gnss);
		mutex.unlock();
	}
	
	bool do_test(){
		try{
			gnssSubscriber = node.subscribe("fix", 10, &GnssDiagnostic::gnssCallback, this);
			
			ros::Time startTime = ros::Time::now();
			double elapsedTime =0.0;
			//loop while condition is not met or exit loop if timer has expired
			while (ros::ok()){
				
				ros::Time currentTime = ros::Time::now();
				elapsedTime = (currentTime - startTime).toSec();
			
				if (receivedMessage.size() >= 1){
					break;
				}
				
				if (elapsedTime >= sleepDuration){
					break;
				}
				ros::Duration(0.1).sleep();
			}
			
			if(receivedMessage.size() > 0){
				this->status = true;
				this->value = std::to_string(receivedMessage.size()) + " message received in " + std::to_string(elapsedTime) + " seconds"; //TODO add more logics to have more info on what is good or not
			}
			
			gnssSubscriber.shutdown();
			receivedMessage.clear();
			return true;
		}
		catch(const std::exception& ex){
			ROS_ERROR_STREAM(ex.what());
			this->status = false;
			this->value = ex.what();
			return false;
		}
	}
	
	
private:
	ros::Subscriber gnssSubscriber;
	ros::NodeHandle node;
	std::mutex mutex;
	double sleepDuration = 3.0;
	std::vector<sensor_msgs::NavSatFix> receivedMessage;

};
#endif
