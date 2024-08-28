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
#include "../../utils/smbus_functions.h"
#include <fcntl.h>
#include <unistd.h>

using namespace std;

class HBV {
	private:
		ros::NodeHandle node;
		ros::Publisher HBVTopic;
		//#define _LL_BUFFSIZE_ 2048

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
		
		double getHumidity(int &file){
			_i2c_smbus_write_quick(file, 0x27);
			usleep(1000);
			
			uint8_t char reg_device1 = 0x00;  // Register to read from in device 1 aka offset
			uint8_t data_device[4];		// Buffer to store the read data
			
			if (_i2c_smbus_read_i2c_block_data(file, reg_device1, sizeof(data_device) , data_device) != 4) {
				//std::cerr << "Vitals Node Failed to read block data from sensor: HIH8130" << std::endl;
				close(file);
				return 1;
			}

			for (int i = 0; i < sizeof(data_device); i++) {
				std::cout << "0x" << std::hex << static_cast<int>(data_device[i]) << " ";
			}
			std::cout << std::endl;
			
			return 0.0;
		}
		
		double getTemp(int &file){
			
		}

		void run(){
			ros::Rate loop_rate(1);
			
			const char* i2c_device = "/dev/i2c-1";
			//int HIH8130_addr = 0x27; //make it a param ??
			
			int file;
			if ((file = open(i2c_device, O_RDWR)) < 0) {
				std::cerr << "Failed to open the I2C bus" << std::endl;
				return 1;
			}
			if (ioctl(file, I2C_SLAVE, 0x27) < 0) {
				std::cerr << "Failed to acquire bus access and/or talk to device 1: " << strerror(errno) << std::endl;
				close(file);
				return 1;
			}
			
			
			while (ros::ok()) {
				raspberrypi_vitals_msg::sysinfo msg;

				msg.header.seq = ++sequenceNumber;
				msg.header.stamp = ros::Time::now();

				msg.cputemp = getCpuTemp();
				msg.cpuload = getCpuLoad();
				msg.freeram = getFreeRam();
				msg.freehdd = getFreeHdd();
				msg.uptime = getUpTime();
				std::cout<< " ok !!!!!!! \n";
				double test = getHumidity(file);
				
				
				msg.vbat = 12.2;
				msg.rh = 25;
				msg.temp = 12;
				msg.psi = 64;

				HBVTopic.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
			close(file);
		}
};

#endif
