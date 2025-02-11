#include "ros/ros.h"
#include <iostream>
#include "i2c_controller_service/i2c_controller_service.h"
#include "logger_service/GetLoggingStatus.h"
#include "HIH8130.h"
#include "INA238.h"
#include "std_msgs/String.h"  // Include for publishing LED state messages

class I2cController {
private:
    ros::NodeHandle n;
    ros::ServiceServer i2cControllerService;
    ros::ServiceClient getLoggingStatusService;
    ros::Publisher led_control_pub;  // ROS publisher for LED control

    HIH8130 weather_sensor;
    INA238 power_sensor;

    ros::Timer ledWarningTimer;
    ros::Timer ledErrorTimer;

    double boardVersion;
    bool (I2cController::*functionVersion)(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res);

    bool set_logger_recording_status() {
        logger_service::GetLoggingStatus status;

        if (!getLoggingStatusService.call(status)) {
            ROS_ERROR("I2cController::set_logger_recording_status() Error while calling GetLoggingStatus service");
            return false;
        }

        std_msgs::String led_msg;

        if (status.response.status) {
            led_msg.data = "recording";
        } else {
            led_msg.data = "ready";
        }

        led_control_pub.publish(led_msg);
        return true;
    }

    void warning_timer_callback(const ros::TimerEvent& event) {
        ROS_INFO("I2cController warning timer expired!");
        std_msgs::String led_msg;
        led_msg.data = "warning";
        led_control_pub.publish(led_msg);
    }

    void error_timer_callback(const ros::TimerEvent& event) {
        ROS_INFO("I2cController error timer expired!");
        std_msgs::String led_msg;
        led_msg.data = "error";
        led_control_pub.publish(led_msg);
    }

    void reset_timer_warning() {
        ledWarningTimer.stop();
        ledWarningTimer = n.createTimer(ros::Duration(5.0), &I2cController::warning_timer_callback, this, true, true);
    }

    void reset_timer_error() {
        ledErrorTimer.stop();
        ledErrorTimer = n.createTimer(ros::Duration(10.0), &I2cController::error_timer_callback, this, true, true);
    }

public:
    I2cController(double &_boardVersion) : boardVersion(_boardVersion) {
        i2cControllerService = n.advertiseService("i2c_controller_service", &I2cController::read_chip, this);
        getLoggingStatusService = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
        getLoggingStatusService.waitForExistence();

        led_control_pub = n.advertise<std_msgs::String>("led_control", 10);  // Advertise LED control topic
    }

    ~I2cController() {}

    bool read_chip(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res) {
        (this->*functionVersion)(req, res);
    }

    bool read_chip_v1(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res) {
        std_msgs::String led_msg;

        if (req.action2perform == "led_error") {
            reset_timer_error();
            led_msg.data = "error";
            led_control_pub.publish(led_msg);
        } 
        else if (req.action2perform == "led_ready") {
            led_msg.data = "ready";
            led_control_pub.publish(led_msg);
        } 
        else if (req.action2perform == "led_recording") {
            led_msg.data = "recording";
            led_control_pub.publish(led_msg);
        } 
        else if (req.action2perform == "led_warning") {
            reset_timer_warning();
            led_msg.data = "warning";
            led_control_pub.publish(led_msg);
        } 
        else if (req.action2perform == "led_nofix") {
            led_msg.data = "nofix";
            led_control_pub.publish(led_msg);
        } 
        else {
            ROS_ERROR_STREAM("I2cController invalid request: " << req.action2perform);
            return false;
        }

        return true;
    }

    bool read_chip_v0(i2c_controller_service::i2c_controller_service::Request &req, i2c_controller_service::i2c_controller_service::Response &res) {
        res.value = -555.0;
        return true;
    }
};
