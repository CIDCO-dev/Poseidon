#include "ros/ros.h"
#include <iostream>
#include "i2c_controller_service/i2c_controller_service.h"
#include "logger_service/GetLoggingStatus.h"
#include "HIH8130.h"
#include "INA238.h"
#include "PCA9533.h"


class I2cController {
private:
	ros::NodeHandle n;
	ros::ServiceServer i2cControllerService;
	ros::ServiceClient getLoggingStatusService;
	
	HIH8130 weather_sensor;
	INA238 power_sensor;
	PCA9533 led_controller;
	
	ros::Timer ledWarningTimer;
	ros::Timer ledErrorTimer;
	
	bool is_recording(){
		logger_service::GetLoggingStatus status;

		if(!getLoggingStatusService.call(status)){
			ROS_ERROR("i2cController Error while calling GetLoggingStatus service");
		}
		
		std::cout<<"is_recording() " << std::boolalpha<< status.response.status<<"\n";
		return status.response.status;
	}
	
	void warning_timer_callback(const ros::TimerEvent& event) {
		ROS_INFO("I2cController warning Timer expired!");
		
		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);
		
		std::cout<<"warning_timer_callback get_led_state response : " << res.value <<"\n";
		
		if(res.value != 4){
			if(is_recording()){
				req.action2perform = "led_recording";
				read_chip(req, res);
			}
			else{
				std::cout<<"set led ready \n";
				req.action2perform = "led_ready";
				read_chip(req, res);
			}
		}
	}

	void error_timer_callback(const ros::TimerEvent& event) {
		ROS_INFO("I2cController error Timer expired!");
		
		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);
		
		if(res.value != 3){
			if(is_recording()){
				req.action2perform = "led_recording";
				read_chip(req, res);
			}
			else{
				req.action2perform = "led_ready";
				read_chip(req, res);
			}
		}
	}
	
	void reset_timer_warning(){
		ledWarningTimer.stop();
		ledWarningTimer = n.createTimer(ros::Duration(10.0), &I2cController::warning_timer_callback, this, true, true);
	}
	void reset_timer_error(){
		ledErrorTimer.stop();
		ledErrorTimer = n.createTimer(ros::Duration(30.0), &I2cController::error_timer_callback, this, true, true);
	}
	
	bool isErrorOn(){
		
		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);
		
		if(res.value == 4){
			return true;
		}
		return false;
	}

public:
	I2cController() {
		i2cControllerService = n.advertiseService("i2c_controller_service", &I2cController::read_chip, this);
		getLoggingStatusService = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
	}

	~I2cController() {
	}
	
	
	
	bool read_chip(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res){
		
		if(req.action2perform == "get_led_state"){
			if(!led_controller.get_state(res)){
				return false;
			}
		}
		
		else if(req.action2perform == "led_error"){
			reset_timer_error();
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
		
		else if(req.action2perform == "led_warning"){
			if(!isErrorOn()){
				reset_timer_warning();
				if(!led_controller.set_led("warning")){
					return false;
				}
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
