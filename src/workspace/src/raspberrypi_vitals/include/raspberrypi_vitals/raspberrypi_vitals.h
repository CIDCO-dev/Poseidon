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
		bool debug_mode = true; // Set to 'false' to disable debug warnings

		float readFloatFromFile(const string& filepath) {
			ifstream file(filepath);
			float value = std::nan("0");
			if (file.is_open()) {
				string content;
				getline(file, content);
				value = strtof(content.c_str(), nullptr);
				file.close();
			} else {
				if (debug_mode) ROS_WARN_STREAM("RPI_Vitals: ❌ Failed to open file: " << filepath);
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
			} else {
				if (debug_mode) ROS_WARN_STREAM("RPI_Vitals: ❌ Failed to read from proc file: " << filepath);
			}
			return value;
		}

		double readMemoryInfo(const string& key) {
			ifstream file("/proc/meminfo");
			if (!file.is_open()) {
				if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to open /proc/meminfo");
				return 0.0;
			}
			string line;
			while (getline(file, line)) {
				if (line.find(key) == 0) {
					line.erase(0, key.length());
					line.erase(line.find("kB"));
					return stod(line) * 1024; // Convert to bytes
				}
			}
			return 0.0;
		}

	public:
		HBV() : sequenceNumber(0) {
			if (debug_mode) ROS_WARN("RPI_Vitals: 🚀 Initializing Raspberry Pi Vitals Node...");
			HBVTopic = node.advertise<raspberrypi_vitals_msg::sysinfo>("vitals", 1000);
			i2c_ctrl_service_client = node.serviceClient<i2c_controller_service::i2c_controller_service>("i2c_controller_service");
			i2c_ctrl_service_client.waitForExistence();
			if (debug_mode) ROS_WARN("RPI_Vitals: ✅ Raspberry Pi Vitals Node Ready!");
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
				if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to run df command");
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

				if (debug_mode) ROS_WARN("RPI_Vitals: 📊 Reading Sensors...");
				if (debug_mode) ROS_WARN("RPI_Vitals: call get_humidity");
				srv.request.action2perform = "get_humidity";
				if (!i2c_ctrl_service_client.call(srv)) {
					if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to read humidity from I2C controller.");
					msg.humidity = -555.0;
				} else {
					msg.humidity = srv.response.value;
				}
				
				if (debug_mode) ROS_WARN("RPI_Vitals: call get_temperature");
				srv.request.action2perform = "get_temperature";
				if (!i2c_ctrl_service_client.call(srv)) {
					if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to read temperature from I2C controller.");
					msg.temperature = -555.0;
				} else {
					msg.temperature = srv.response.value;
				}

				if (debug_mode) ROS_WARN("RPI_Vitals: call get_voltage");
				srv.request.action2perform = "get_voltage";
				if (!i2c_ctrl_service_client.call(srv)) {
					if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to read voltage from I2C controller.");
					msg.voltage = -555.0;
				} else {
					msg.voltage = srv.response.value;
				}

				if (debug_mode) ROS_WARN("RPI_Vitals: call led state");
				srv.request.action2perform = "get_led_state";
				if (!i2c_ctrl_service_client.call(srv)) {
				} else {
					msg.ledstate = srv.response.value;
				}

				std::pair<bool, std::string> criticalResult = isCritical(msg);
				std::pair<bool, std::string> warningResult = isWarning(msg);

				if (criticalResult.first) {
					srv.request.action2perform = "led_error";
					if (!i2c_ctrl_service_client.call(srv)) {
						if (debug_mode) ROS_WARN("RPI_Vitals: ❌ Failed to send ERROR LED command.");
					}
					msg.status = criticalResult.second;
				} else if (warningResult.first) {
					srv.request.action2perform = "led_warning";
					if (!i2c_ctrl_service_client.call(srv)) {
					}
					msg.status = warningResult.second;
				} else {
					msg.status = "Normal";
				}


				HBVTopic.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		std::pair<bool, std::string> isCritical(raspberrypi_vitals_msg::sysinfo &msg) {
			if (msg.freehdd < 5.0 && msg.freehdd > 0.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [CRITICAL] Hard Disk Full detected (free HDD: %.2f%%)", msg.freehdd);
				}
				return std::make_pair(true, "Hard Disk Full");
			}
			if (msg.voltage <= 11.3 && msg.voltage > 0.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [CRITICAL] Battery Under Voltage detected (voltage: %.2fV)", msg.voltage);
				}
				return std::make_pair(true, "Battery Under Voltage");
			}
			if (msg.voltage >= 13.8) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [CRITICAL] Battery Overvoltage detected (voltage: %.2fV)", msg.voltage);
				}
				return std::make_pair(true, "Battery Overvoltage");
			}
			if (msg.cputemp >= 80.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [CRITICAL] CPU Over Temperature detected (CPU Temp: %.2f°C)", msg.cputemp);
				}
				return std::make_pair(true, "CPU Over Temperature");
			}
			return std::make_pair(false, "Normal");
		}
		
		std::pair<bool, std::string> isWarning(raspberrypi_vitals_msg::sysinfo &msg) {
			if (msg.freehdd < 20.0 && msg.freehdd > 0.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [WARNING] Low Hard Disk Space detected (free HDD: %.2f%%)", msg.freehdd);
				}
				return std::make_pair(true, "Low Hard Disk Space");
			}
			if (msg.voltage <= 11.9 && msg.voltage > 0.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [WARNING] Battery Low Voltage detected (voltage: %.2fV)", msg.voltage);
				}
				return std::make_pair(true, "Battery Low Voltage");
			}
			if (msg.cputemp >= 75.0) {
				if (debug_mode) {
					ROS_WARN("RPI_Vitals: [WARNING] High CPU Temperature detected (CPU Temp: %.2f°C)", msg.cputemp);
				}
				return std::make_pair(true, "High CPU Temperature");
			}
			return std::make_pair(false, "Normal");
		}
		
};

#endif
