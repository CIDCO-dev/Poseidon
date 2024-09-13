#include <iostream>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdint>
#include "ros/ros.h"

class HIH8130 {
public:
	HIH8130(){
		if ((fileDescriptor = open(i2cDevice, O_RDWR)) < 0) {
			ROS_ERROR("HIH8130 Failed to open the I2C bus");
			exit(1);
		}

		if (ioctl(fileDescriptor, I2C_SLAVE, deviceAddress) < 0) {
			ROS_ERROR("Failed to connect to the HIH8130 sensor");
			exit(1);
		}
	}

	bool get_humidity(i2c_controller_service::i2c_controller_service::Response &response) {
		uint8_t data[4];
		
			if (write(fileDescriptor, &deviceAddress, 1) != 1) {
				ROS_ERROR("HIH8130::get_humidity() Failed to write");
				return false;
			}

			usleep(100000);  // 100 ms

			if (read(fileDescriptor, data, 4) != 4) {
				ROS_ERROR("HIH8130::get_humidity() Failed to read humidity");
				return false;
			}
		
		uint16_t humidityRaw = ((data[0] & 0x3F) << 8) + data[1];
		response.value = (humidityRaw / (double)((1 << 14) - 2)) * 100.0;
		return true;
	}
	
	bool get_temperature(i2c_controller_service::i2c_controller_service::Response &response){
		uint8_t data[4];
		
			if (write(fileDescriptor, &deviceAddress, 1) != 1) {
				ROS_ERROR("HIH8130::get_temperature() Failed to write");
				return false;
			}

			usleep(100000);  // 100 ms

			if (read(fileDescriptor, data, 4) != 4) {
				ROS_ERROR("HIH8130::get_temperature() Failed to read temperature");
				return false;
			}
		
		uint16_t tempRaw = (data[2] << 6) + (data[3] >> 2);
		response.value = ((tempRaw / (double)((1 << 14) - 2)) * 165.0) - 40.0;
		return true;
		
	}

	~HIH8130() {
		if (fileDescriptor >= 0) {
			close(fileDescriptor);
		}
	}

private:
	int fileDescriptor;
	uint8_t deviceAddress = 0x27;
	const char* i2cDevice = "/dev/i2c-1";
};

