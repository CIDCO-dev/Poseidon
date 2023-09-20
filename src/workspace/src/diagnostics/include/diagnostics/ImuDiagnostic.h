#ifndef IMuDIAGNOISTIC_H
#define IMUDIAGNOISTIC_H

#include "DiagnosticsTest.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "../../utils/QuaternionUtils.h"

class ImuDiagnostic : public DiagnosticsTest{
	
public:
	ImuDiagnostic(std::string name, std::string topic, int messageFrequency): DiagnosticsTest(name, topic, messageFrequency), transformListener(buffer){}
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
		
		
		double heading = 0;
		double pitch   = 0;
		double roll    = 0;
		
		geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));
		QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation, imu.orientation, heading, pitch, roll);
		
		if(std::abs(pitch) < 1.5 && std::abs(roll) < 1.5){
			imuCalibration++;
		}
	}
	
	bool is_calibrated(){
		
		if(!(imuCalibration > 0)){
			this->value += "Not calibrated \n";
			return false;
		}
		
		double calibratedMsgPourcent = ((double)imuCalibration / (double)messageCount) * 100;
		
		if(calibratedMsgPourcent > 95){
			this->value += std::to_string(calibratedMsgPourcent) + "% " + "of messages angles are calibrated \n";
			return true;
		}
		else{
			this->value += std::to_string(calibratedMsgPourcent) + "% " + "of messages angles are calibrated \n";
			return false;
		}
	}
	
	void start_subscriber() override{
		//reinitialize test results
		this->status = false;
		this->value = "";
		messageCount = 0;
		imuCalibration = 0;
		
		subscriber = node.subscribe(this->topic, 10, &ImuDiagnostic::messageCallback, this);
	}
	
	void do_tests()override{
		
		if(is_receiving_messages() && is_calibrated()){ //add other test here
			this->status = true;
		}
		else{
			this->status = false;
		}
	}
	
private:
	//std::vector<sensor_msgs::Imu> receivedMessages;
	int imuCalibration = 0;
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener transformListener;

};
#endif
