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
	
		int data=0;
		
		if(!readRegister(REGISTER_VOLTAGE, data)){
			return false;
		}
		
		response.value = (data * 3.125) / 1000.0;
		return true;
	}

	bool readShuntVoltage(i2c_controller_service::i2c_controller_service::Response &response) {
	
		int data = 0;
		
		if(!readRegister(REGISTER_SHUNT, data)){
			return false;
		}
		
		response.value = (data * 5.0) / 1000.0;  
		return true;
	}

	bool readTemperature(i2c_controller_service::i2c_controller_service::Response &response) {
	
		int data = 0;
		
		if(!readRegister(REGISTER_TEMPERATURE, data)){
			return false;
		}
		
		int raw_temperature = data >> 4;  

		if (raw_temperature & 0x800) {
			raw_temperature = -((~raw_temperature & 0xFFF) + 1);
		}

		response.value = (raw_temperature * 125.0) / 1000.0;
		return true;
	}

private:
	const char* i2c_device = "/dev/i2c-1";
	static constexpr int ina238_address = 0x40;
	int file;

	static constexpr int REGISTER_VOLTAGE = 0x05;
	static constexpr int REGISTER_SHUNT = 0x04;
	static constexpr int REGISTER_TEMPERATURE = 0x06;

	bool readRegister(int reg, int data) {
		uint8_t buffer[2];

		if (write(file, &reg, 1) != 1) {
			ROS_ERROR("Failed to write to the I2C bus.");
			return false;
		}

		if (read(file, buffer, 2) != 2) {
			ROS_ERROR("Failed to read from the I2C bus.");
			return false;
		}

		data = (buffer[0] << 8) | buffer[1];
		return true;
	}
};

