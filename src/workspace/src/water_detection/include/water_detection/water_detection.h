#include <wiringPi.h>
#include <iostream>
#include "ros/ros.h"
#include "i2c_controller_service/i2c_controller_service.h"

class WaterDetector {
public:
	WaterDetector(bool _inverseImmersionSignal = false): inverseImmersionSignal(_inverseImmersionSignal) {
		if (wiringPiSetup() == -1) {
			ROS_ERROR("WiringPi initialization failed!");
			exit(1);
		}	
		pinMode(waterInfiltrationPin, INPUT);
		pinMode(immersionPin, INPUT);
		i2c_ctrl_service_client  = node.serviceClient<i2c_controller_service::i2c_controller_service>("i2c_controller_service");
		i2c_ctrl_service_client.waitForExistence();
		
	}
	
	~WaterDetector(){}
	
	bool is_immersed(){
		if(inverseImmersionSignal){
			return !digitalRead(immersionPin);
		}
		else{
			return digitalRead(immersionPin);
		}
		
	}
	
	bool has_water_infiltration(){
		return digitalRead(waterInfiltrationPin);
	}
	
	void run(){
		ros::Rate loop_rate(10);
		while (ros::ok()) {
			
			//TODO
			//if(is_immersed()){
				
			//}
			
			if(has_water_infiltration()){
				ROS_ERROR("water infiltration detected");
				srv.request.action2perform = "led_error";
				if(!i2c_ctrl_service_client.call(srv)){
					ROS_ERROR("Rapberrypi vitals run(), I2C controller service call failed: led_error");
				}
			}
			
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

private:
	int waterInfiltrationPin = 18;
	int immersionPin = 17;
	bool inverseImmersionSignal = false;
	ros::NodeHandle node;
	ros::ServiceClient i2c_ctrl_service_client;
	i2c_controller_service::i2c_controller_service srv;
};

