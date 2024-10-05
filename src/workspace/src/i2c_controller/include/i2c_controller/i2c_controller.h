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
	
	
	bool set_logger_recording_status(){
		logger_service::GetLoggingStatus status;

		if(!getLoggingStatusService.call(status)){
			ROS_ERROR("I2cController::is_logger_recording() Error while calling GetLoggingStatus service");
			return false;
		}
		
		if(status.response.status){
				if(!led_controller.set_led("recording")){
					ROS_ERROR("1-set_logger_recording_status failed call to i2c_controller_service");
					return false;
				}
			}
			else{
				if(!led_controller.set_led("ready")){
					ROS_ERROR("2-set_logger_recording_status failed call to i2c_controller_service");
					return false;
				}
			}
		return true;
	}
	
	/*
		In order for the timer callbacks to override the led state 
		we have to bypass the readChip function, because of the isIssue function
		that prevent a led state change triggered by the user if he would be using the manual logging mode
	*/
	
	void warning_timer_callback(const ros::TimerEvent& event) {
		ROS_INFO("I2cController warning Timer expired!");
		
		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);
		
		if(res.value <= 3){
			if(!set_logger_recording_status()){
				ROS_ERROR("i2cController warning timer callback could not set led state");
			}
		}
	}

	void error_timer_callback(const ros::TimerEvent& event) {
		ROS_INFO("I2cController error Timer expired!");
		
		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);
		
		if(res.value != 3 && res.value != 4){
			if(!set_logger_recording_status()){
				ROS_ERROR("i2cController error timer callback could not set led state");
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

		
		if(res.value == 5){
			return true;
		}
		return false;
	}
	
	bool isNoFixOn(){

		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);

		
		if(res.value == 4){
			return true;
		}
		return false;
	}
	
	bool isWarningOn(){

		i2c_controller_service::i2c_controller_service::Request req;
		i2c_controller_service::i2c_controller_service::Response res;
		
		req.action2perform = "get_led_state";
		read_chip(req, res);

		if(res.value == 3){
			return true;
		}
		return false;
	}

public:
	I2cController() {
		i2cControllerService = n.advertiseService("i2c_controller_service", &I2cController::read_chip, this);
		getLoggingStatusService = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
		getLoggingStatusService.waitForExistence();
	}

	~I2cController() {
	}
	
	
	
	bool read_chip(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res){
		
		//ROS_INFO_STREAM("i2cController::read_chip() : " << req.action2perform);
		
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
			if(!isErrorOn() && !isNoFixOn() && !isWarningOn()){ // to prevent led state change by the user from UI
				if(!led_controller.set_led("ready")){
					return false;
				}
			}
		}
		
		else if(req.action2perform == "led_recording"){
			if(!isErrorOn() && !isNoFixOn() && !isWarningOn()){ // to prevent led state change by the user from UI
				if(!led_controller.set_led("recording")){
					return false;
				}
			}
		}
		
		else if(req.action2perform == "led_warning"){
			if(!isErrorOn() && !isNoFixOn()){
				reset_timer_warning();
				if(!led_controller.set_led("warning")){
					return false;
				}
			}
		}
		
		else if(req.action2perform == "led_nofix"){
			if(!isErrorOn()){
				if(!led_controller.set_led("nofix")){
					return false;
				}
			}
		}
		else if(req.action2perform == "gps_fix_ready"){
			if(!isErrorOn() && !isWarningOn()){
				if(!led_controller.set_led("ready")){
					return false;
				}
			}
		}
		else if(req.action2perform == "gps_fix_recording"){
			if(!isErrorOn() && !isWarningOn()){
				if(!led_controller.set_led("recording")){
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
