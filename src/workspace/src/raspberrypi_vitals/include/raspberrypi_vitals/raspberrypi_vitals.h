#ifndef raspberrypi_vitals
#define raspberrypi_vitals

#include "ros/ros.h"
#include "raspberrypi_vitals_msg/sysinfo.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include "HIH8130_sensor.h"

using namespace std;

class HBV {
	private:
		ros::NodeHandle node;
		ros::Publisher HBVTopic;

		uint32_t sequenceNumber;
		float upt;

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
			
			const char* i2cDevice = "/dev/i2c-1";
			uint8_t sensorAddress = 0x27;
			
			HIH8130 sensor(i2cDevice, sensorAddress);
			
			
			while (ros::ok()) {
				raspberrypi_vitals_msg::sysinfo msg;

				msg.header.seq = ++sequenceNumber;
				msg.header.stamp = ros::Time::now();
				
				
				msg.cputemp = getCpuTemp();
				msg.cpuload = getCpuLoad();
				msg.freeram = getFreeRam();
				msg.freehdd = getFreeHdd();
				msg.uptime = getUpTime();
				msg.humidity = sensor.get_humidity();
				msg.temperature = sensor.get_temperature();
				
				
				msg.vbat = 12.2;
				msg.rh = 25;
				msg.temp = 12;
				msg.psi = 64;

				HBVTopic.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
};

#endif
