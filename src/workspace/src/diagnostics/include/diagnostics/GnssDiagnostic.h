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
			auto start = std::chrono::steady_clock::now();
			while(std::chrono::steady_clock::now() - start < std::chrono::seconds(time)){
				//wait
			}
			
			if(receivedMessage.size() > 0){
				this->status = true;
				this->value = std::to_string(receivedMessage.size()) + " message received in " + std::to_string(time) + " seconds"; //TODO add more logics to have more info on what is good or not
			}
			
			gnssSubscriber.shutdown();
			receivedMessage.clear();
			return true;
		}
		catch(const std::exception& ex){
			ROS_ERROR_STREAM(ex.what());
			return false;
		}
	}
	
	
private:
	ros::Subscriber gnssSubscriber;
	ros::NodeHandle node;
	std::mutex mutex;
	int time = 10;
	std::vector<sensor_msgs::NavSatFix> receivedMessage;

};
#endif
