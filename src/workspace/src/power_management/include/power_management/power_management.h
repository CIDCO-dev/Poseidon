#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdexcept>
#include <cstdint>

class PowerManagement {
public:
	PowerManagement(const char* i2c_device){
		// Open the I2C bus
		if ((file = open(i2c_device, O_RDWR)) < 0) {
			throw std::runtime_error("Failed to open the I2C bus.");
		}

		// Specify the address of the I2C device
		if (ioctl(file, I2C_SLAVE, ina238_address) < 0) {
			throw std::runtime_error("Failed to acquire bus access and/or talk to slave.");
		}
	}

	~PowerManagement() {
		if (file >= 0) {
			close(file);
		}
	}

	double readVoltage() {
		int data = readRegister(REGISTER_VOLTAGE);
		double voltage = (data * 3.125) / 1000.0;
		return voltage;
	}

	double readShuntVoltage() {
		int data = readRegister(REGISTER_SHUNT);
		double shunt_voltage = (data * 5.0) / 1000.0;  
		return shunt_voltage;
	}

	double readTemperature() {
		int data = readRegister(REGISTER_TEMPERATURE);
		int raw_temperature = data >> 4;  

		if (raw_temperature & 0x800) {
			raw_temperature = -((~raw_temperature & 0xFFF) + 1);
		}

		double temperature = (raw_temperature * 125.0) / 1000.0;
		return temperature;
	}
	
	void run(){
		ros::Rate loop_rate(1);
		while (ros::ok()) {
				
				try{
				
					ROS_INFO_STREAM(readVoltage());
				
				}
				catch (const std::exception &e) {
					ROS_ERROR_STREAM("power_management.h run Error: " << e.what());
				}
				
				ros::spinOnce();
				loop_rate.sleep();
	}

private:
	static constexpr int ina238_address = 0x40;
	int file;

	static constexpr int REGISTER_VOLTAGE = 0x05;
	static constexpr int REGISTER_SHUNT = 0x04;
	static constexpr int REGISTER_TEMPERATURE = 0x06;

	int readRegister(int reg) {
		uint8_t buffer[2];

		if (write(file, &reg, 1) != 1) {
			throw std::runtime_error("Failed to write to the I2C bus.");
		}

		if (read(file, buffer, 2) != 2) {
			throw std::runtime_error("Failed to read from the I2C bus.");
		}

		return (buffer[0] << 8) | buffer[1];
	}
};

