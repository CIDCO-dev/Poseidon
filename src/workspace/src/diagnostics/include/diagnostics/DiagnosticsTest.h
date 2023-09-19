#ifndef DIAGNOISTICSTEST_H
#define DIAGNOISTICSTEST_H

#include <iostream>
#include <string>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include "geometry_msgs/PointStamped.h"
/*#include "nav_msgs/Odometry.h"*/
/*#include "std_msgs/String.h"*/

class DiagnosticsTest {
public:

	DiagnosticsTest(std::string name, std::string _topic, int _messageFrequency):key(name), topic(_topic), messageFrequency(_messageFrequency){}
	virtual ~DiagnosticsTest() {}
	
	virtual void do_tests() = 0;
	virtual void start_subscriber() = 0;
	
	virtual bool do_latency_test(){
		try{
			start_subscriber();
			
			ros::Time startTime = ros::Time::now();
			double elapsedTime = 0.0;
			//loop while condition is not met or exit loop if timer has expired
			while (ros::ok()){
				
				ros::Time currentTime = ros::Time::now();
				elapsedTime = (currentTime - startTime).toSec();
			
				if (messageCount >= (latencySleepDuration * messageFrequency)){
					break;
				}
				
				if (elapsedTime >= latencySleepDuration){
					break;
				}
				ros::Duration(0.1).sleep();
			}
			
			if(messageCount > 0){
				if( ( ((double)messageCount / messageFrequency) / latencySleepDuration)*100 >= 90.0 ){
					this->status = true;
				}
				else{
					this->status = false;
				}
				this->value = std::to_string(messageCount) + " message received in " + std::to_string(elapsedTime) + " seconds";
			}
			else{
				this->status = false;
				this->value = std::to_string(messageCount) + " message received in " + std::to_string(elapsedTime) + " seconds";
			}
			
			subscriber.shutdown();
			return this->status;
		}
		catch(const std::exception& ex){
			ROS_ERROR_STREAM(ex.what());
			this->status = false;
			this->value = ex.what();
			subscriber.shutdown();
			return false;
		}
	}
	
	virtual bool is_receiving_messages(){
		this->status = false;
		try{
			start_subscriber();
			
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
			return this->status;
		}
		catch(const std::exception& ex){
			ROS_ERROR_STREAM(ex.what());
			this->status = false;
			this->value = ex.what();
			subscriber.shutdown();
			return false;
		}
	
	}
	
	virtual void to_json(rapidjson::Document &doc, rapidjson::Value &diagnosticsArray){
	
		rapidjson::Document diagnostic(rapidjson::kObjectType);
		
		rapidjson::Value diagName(this->key.c_str(), doc.GetAllocator());
		diagnostic.AddMember("name", diagName, doc.GetAllocator());
		
		rapidjson::Value infoStr(this->value.c_str(), doc.GetAllocator());
		diagnostic.AddMember("message", infoStr, doc.GetAllocator());
		
		rapidjson::Value status(this->status);
		diagnostic.AddMember("status", status, doc.GetAllocator());

		diagnosticsArray.PushBack(diagnostic, doc.GetAllocator());
	}

protected:
	std::string key;
	std::string value;
	bool status = false;
	ros::Subscriber subscriber;
	ros::NodeHandle node;
	std::mutex mutex;
	double sleepDuration = 1.5;
	double latencySleepDuration = 10.5;
	int messageCount = 0;
	std::string topic;
	int messageFrequency;
	
};
#endif
