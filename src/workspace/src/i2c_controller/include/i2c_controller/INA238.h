#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdexcept>
#include <cstdint>

class INA238 {
public:
	INA238(){
		// Open the I2C bus
		if ((file = open(i2c_device, O_RDWR)) < 0) {
			ROS_ERROR("Failed to open the I2C bus.");
		}

		// Specify the address of the I2C device
		if (ioctl(file, I2C_SLAVE, ina238_address) < 0) {
			ROS_ERROR("Failed to acquire bus access and/or talk to slave.");
		}
	}

	~INA238() {
		if (file >= 0) {
			close(file);
		}
	}

	bool readVoltage(i2c_controller_service::i2c_controller_service::Response &response) {
	
		uint8_t data[2];
		
		if(!readRegister(REGISTER_VOLTAGE, data)){
			return false;
		}
		
		response.value = (data[0] * 256 + data[1]) * 3.125 / 1000;
		return true;
	}

	bool readShuntVoltage(i2c_controller_service::i2c_controller_service::Response &response) {
	
		uint8_t data[2];
		
		if(!readRegister(REGISTER_SHUNT, data)){
			return false;
		}
		
		response.value = (data[0] * 256 + data[1]) * 5.0 / 1000.0;
		return true;
	}

	bool readTemperature(i2c_controller_service::i2c_controller_service::Response &response) {
	
		uint8_t data[2];
		
		if(!readRegister(REGISTER_TEMPERATURE, data)){
			return false;
		}
		
		int raw_temperature = (data[0] * 256 + data[1]) >> 4;  

		if (raw_temperature & 0x800) {
			raw_temperature = -((~raw_temperature & 0xFFF) + 1);
		}

		response.value = (raw_temperature * 125.0) / 1000.0;
		return true;
	}

private:
	const char* i2c_device = "/dev/i2c-4";
	static constexpr int ina238_address = 0x40;
	int file;

	int REGISTER_VOLTAGE = 0x05;
	int REGISTER_SHUNT = 0x04;
	int REGISTER_TEMPERATURE = 0x06;

	
	bool readRegister(const int &reg, uint8_t (&data)[2]) {
		
		if (write(file, &reg, 1) != 1) {
			ROS_ERROR("Failed to write to the I2C bus.");
			return false;
		}

		if (read(file, data, 2) != 2) {
			ROS_ERROR("Failed to read from the I2C bus.");
			return false;
		}
		
		
		return true;
	}
};

