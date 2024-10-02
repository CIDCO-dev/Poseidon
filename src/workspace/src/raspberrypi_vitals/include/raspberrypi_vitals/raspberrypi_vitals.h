#ifndef raspberrypi_vitals
#define raspberrypi_vitals

#include "ros/ros.h"
#include "ros/console.h"
#include "raspberrypi_vitals_msg/sysinfo.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include "i2c_controller_service/i2c_controller_service.h"
#include "power_management_msg/batteryMsg.h"
#include "logger_service/ToggleLogging.h"

using namespace std;

class HBV {
	private:
		ros::NodeHandle node;
		ros::Publisher HBVTopic;
		ros::Subscriber batteryTopic;
		ros::ServiceClient i2c_ctrl_service_client;
		i2c_controller_service::i2c_controller_service srv;
		ros::ServiceClient loggerServiceClient;
		logger_service::ToggleLogging loggerService;

		uint32_t sequenceNumber;
		float upt;
		double batteryVoltage = 0.0;

		float readFloatFromFile(const string& filepath) {
			ifstream file(filepath);
			float value = std::nan("0");
			if (file.is_open()) {
				string content;
				getline(file, content);
				value = strtof(content.c_str(), nullptr);
				file.close();
			}
			return value;
		}

		double readDoubleFromProc(const string& filepath) {
			ifstream file(filepath);
			double value = std::nan("0");
			if (file.is_open()) {
				string content;
				getline(file, content);
				value = stod(content);
				file.close();
			}
			return value;
		}

		double readMemoryInfo(const string& key) {
			ifstream file("/proc/meminfo");
			if (file.is_open()) {
				string line;
				while (getline(file, line)) {
					if (line.find(key) == 0) {
						line.erase(0, key.length());
						line.erase(line.find("kB"));
						return stod(line) * 1024; // Convert to bytes
					}
				}
				file.close();
			}
			return 0.0;
		}
		

	public:
		HBV() : sequenceNumber(0) {
			HBVTopic = node.advertise<raspberrypi_vitals_msg::sysinfo>("vitals", 1000);
			i2c_ctrl_service_client  = node.serviceClient<i2c_controller_service::i2c_controller_service>("i2c_controller_service");
			batteryTopic = node.subscribe("batteryStatus", 1000, &HBV::batteryCallback, this);
			
		}
		
		void batteryCallback(const power_management_msg::batteryMsg& msg){
			batteryVoltage = msg.voltage;
		}
		
		float getCpuTemp() {
			return readFloatFromFile("/sys/class/thermal/thermal_zone0/temp") / 1000.0;
		}

		double getCpuLoad() {
			double loadavg = readDoubleFromProc("/proc/loadavg");
			return (loadavg / 8.0) * 100.0;
		}

		double getFreeRam() {
			double totalram = readMemoryInfo("MemTotal:");
			double freeram = readMemoryInfo("MemFree:");
			return (freeram / totalram) * 100.0;
		}

		float getUpTime() {
			return readFloatFromFile("/proc/uptime");
		}

		double getFreeHdd() {
			FILE *fp;
			char path[1035];
			double free_hdd = std::nan("0");

			fp = popen("df --output=pcent / | tail -n 1", "r");
			if (fp == NULL) {
				ROS_WARN("Failed to run df command");
				return free_hdd;
			}

			if (fgets(path, sizeof(path), fp) != NULL) {
				string output = path;
				output.erase(output.find('%'));
				free_hdd = 100.0 - stod(output);
			}

			pclose(fp);
			return free_hdd;
		}

		void run(){
			ros::Rate loop_rate(1);
			
			while (ros::ok()) {
				raspberrypi_vitals_msg::sysinfo msg;

				msg.header.seq = ++sequenceNumber;
				msg.header.stamp = ros::Time::now();
				
				
				msg.cputemp = getCpuTemp();
				msg.cpuload = getCpuLoad();
				msg.freeram = getFreeRam();
				msg.freehdd = getFreeHdd();
				msg.uptime = getUpTime();
				
				
				srv.request.action2perform = "get_humidity";
				if(i2c_ctrl_service_client.call(srv)){
					msg.humidity = srv.response.value;
				}
				else{
					ROS_ERROR("1- vitals could not call i2c controller");
					msg.humidity = 100.0;
				}
				
				srv.request.action2perform = "get_temperature";
				if(i2c_ctrl_service_client.call(srv)){
					msg.temperature = srv.response.value;
				}
				else{
					ROS_ERROR("2- vitals could not call i2c controller");
					msg.temperature = 100.0;
				}
				
				msg.voltage = batteryVoltage;
				msg.vbat = 12.2;
				msg.rh = 25;
				msg.psi = 64;
				
				if(isCritical(msg)){
					srv.request.action2perform = "led_error";
					if(!i2c_ctrl_service_client.call(srv)){
						ROS_ERROR("Rapberrypi vitals run(), I2C controller service call failed: led_warning");
					}
				}
				else if(isWarning(msg)){
					srv.request.action2perform = "led_warning";
					if(!i2c_ctrl_service_client.call(srv)){
						ROS_ERROR("Rapberrypi vitals run(), I2C controller service call failed: led_error");
					}
				}
				
				HBVTopic.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		
	bool isCritical(raspberrypi_vitals_msg::sysinfo &msg){
		
		if(msg.freehdd < 1.0){
			loggerService.request.loggingEnabled = false;
			if(loggerServiceClient.call(loggerService)){
				if(loggerService.response.loggingStatus){
					ROS_ERROR("Rapberrypi vitals isCritical(), could not turn off logger");
				}
			}
			else{
				ROS_ERROR("Rapberrypi vitals isCritical(), logger service call failed");
			}
			
			return true;
		}
		else if(msg.voltage <= 11.0 || msg.voltage >= 13.0){
			return true;
		}
		/*
			The CPU temperature on a Raspberry Pi must stay below 85 °C to keep it running with the best performance. 
			The CPU will slow down (throttle) as it approaches this threshold, which can lead to a general slowness of the operating system.
			https://raspberrytips.com/raspberry-pi-temperature/
		*/
		else if(msg.cputemp >= 80.0){
			return true;
		}
		
		
		return false;
	}
	
	bool isWarning(raspberrypi_vitals_msg::sysinfo &msg){
		if(msg.freehdd < 5.0){
			return true;
		}
		else if(msg.voltage <= 11.9){
			return true;
		}
		else if(msg.cputemp >= 75.0){
			return true;
		}
		
		return false;
	}
};

#endif
