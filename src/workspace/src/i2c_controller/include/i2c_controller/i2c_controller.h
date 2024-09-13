#include "ros/ros.h"
#include <iostream>
#include "i2c_controller_service/i2c_controller_service.h"
#include "HIH8130.h"
#include "INA238.h"
#include "PCA9533.h"


class I2cController {
private:
	ros::NodeHandle n;
	ros::ServiceServer i2cControllerService;
	
	//XXX
	HIH8130 weather_sensor;
	INA238 power_sensor;
	PCA9533 led_controller;

public:
	I2cController() {
		
		i2cControllerService = n.advertiseService("i2c_controller_service", &I2cController::read_chip, this);
		

	}

	~I2cController() {
	}
	
	
	
	bool read_chip(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res){
		
		if(req.action2perform == "led_error"){
			if(!led_controller.set_led("error")){
				return false;
			}
		}
		
		else if(req.action2perform == "led_ready"){
			if(!led_controller.set_led("ready")){
				return false;
			}
		}
		
		else if(req.action2perform == "led_recording"){
			if(!led_controller.set_led("recording")){
				return false;
			}
		}
		
		else if(req.action2perform == "get_humidity"){
			if(!weather_sensor.get_humidity(res)){
				return false;
			}
		}
		
		else if(req.action2perform == "get_temperature"){
			if(!weather_sensor.get_temperature(res)){
				return false;
			}
		}
		
		else if(req.action2perform == "get_voltage"){
			if(!power_sensor.readVoltage(res)){
				return false;
			}
		}
		
		else if(req.action2perform == "get_shuntVoltage"){
			if(!power_sensor.readShuntVoltage(res)){
				return false;
			}
		}
		
		else if(req.action2perform == "get_batteryTemperature"){
			if(!power_sensor.readTemperature(res)){
				return false;
			}
		}
		
		else{
			ROS_ERROR_STREAM("i2c controller invalid request: " << req.action2perform);
			return false;
		}
		
		return true;
	}

};
