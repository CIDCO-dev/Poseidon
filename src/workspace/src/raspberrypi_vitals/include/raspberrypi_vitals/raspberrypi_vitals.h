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
			i2c_ctrl_service_client.waitForExistence();
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
					//ROS_ERROR("1- vitals could not call i2c controller");
					msg.humidity = -555.0;
				}
				
				srv.request.action2perform = "get_temperature";
				if(i2c_ctrl_service_client.call(srv)){
					msg.temperature = srv.response.value;
				}
				else{
					//ROS_ERROR("2- vitals could not call i2c controller");
					msg.temperature = -555.0;
				}
				
				srv.request.action2perform = "get_voltage";
				if(i2c_ctrl_service_client.call(srv)){
					msg.voltage = srv.response.value;
				}
				else{
					//ROS_ERROR("3- vitals could not call i2c controller");
					msg.voltage = -555.0;
				}
				
				std::pair<bool, std::string> criticalResult = isCritical(msg);
				std::pair<bool, std::string> warningResult = isWarning(msg);

				if(criticalResult.first){
					srv.request.action2perform = "led_error";
					if(!i2c_ctrl_service_client.call(srv)){
						//ROS_ERROR("Rapberrypi vitals run(), I2C controller service call failed: led_warning");
						
					}
					msg.status = criticalResult.second;
				}
				
				else if(warningResult.first){
					srv.request.action2perform = "led_warning";
					if(!i2c_ctrl_service_client.call(srv)){
						//ROS_ERROR("Rapberrypi vitals run(), I2C controller service call failed: led_error");
						
					}
					msg.status = warningResult.second;
				}

				srv.request.action2perform = "get_led_state";
				if(i2c_ctrl_service_client.call(srv)){
					msg.ledstate = srv.response.value;
				}
				

				HBVTopic.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		
		
	/*
		for the function isCritical ans isWarning each threshold changes needs to be reflected in the UI
	*/
	std::pair<bool, std::string> isCritical(raspberrypi_vitals_msg::sysinfo &msg){
		
		/*
			In the case scenario that the hardware is not eqquipped with a newer board version
			no led warning or critical warning shoul be triggered
		*/
		
		/*
			the logger will stop recording data if freehdd is < 1%
			this gives 4% of hdd time acquisition for the user to react accordingly
		*/
		if(msg.freehdd < 5.0 && msg.freehdd > 0.0){
			//ROS_ERROR("isCritical free hdd < 5 pourcent");
			return std::make_pair(true, "Hard Disk Full");
		}
		else if(msg.voltage <= 11.3 && msg.voltage > 0.0){
			//ROS_ERROR_STREAM("isCritical voltage <= 11.3v || voltage >= 13v	voltage: "  << msg.voltage <<"v");
			return std::make_pair(true, "Battery Under Voltage Detected");
		}
		else if(msg.voltage >= 13.8){
			//ROS_ERROR_STREAM("isCritical voltage <= 11.3v || voltage >= 13v	voltage: "  << msg.voltage <<"v");
			return std::make_pair(true, "Battery Overvoltage Detected");
		}
		/*
			The CPU temperature on a Raspberry Pi must stay below 85 °C to keep it running with the best performance.
			The CPU will slow down (throttle) as it approaches this threshold, which can lead to a general slowness of the operating system.
			https://raspberrytips.com/raspberry-pi-temperature/
		*/
		else if(msg.cputemp >= 80.0){
			//ROS_ERROR("isCritical cpu temp >= 80 C");
			return std::make_pair(true, "CPU Over Temperature");
		}
		
		return std::make_pair(false, "Normal");
	}
	
	
	/*
		In the case scenario that the hardware is not eqquipped with a newer board version
		no led warning or critical warning shoul be triggered
	*/
	std::pair<bool, std::string> isWarning(raspberrypi_vitals_msg::sysinfo &msg){
		if(msg.freehdd < 20.0 && msg.freehdd > 0.0){
			//ROS_WARN("isWarning free hdd < 20 pourcent");
			return std::make_pair(true, "Low Hard Disk Space");
		}
		else if(msg.voltage <= 11.9 && msg.voltage > 0.0){
			//ROS_WARN("isWarning voltage <= 11.9 v");
			return std::make_pair(true, "Battery Low Volgage Detected");
		}
		else if(msg.cputemp >= 75.0){
			//ROS_WARN("isWarning cpu temp >= 75 C");
			return std::make_pair(true, "CPU Higi Temperature");
		}
		
		return std::make_pair(false, "Normal");;
	}
};

#endif
