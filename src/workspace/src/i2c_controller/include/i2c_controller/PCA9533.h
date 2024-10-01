#include "ros/ros.h"
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

			// 0,1,2,3,4,5
enum States {Off, Ready, Recording, Warning, NoFix, Critical};

class PCA9533 {
private:
	const int I2C_ADDR = 0x62;
	const char *I2C_DEVICE = "/dev/i2c-4"; 
	int file;
	
	// PCA9533 Registers
	const uint8_t PSC0[2] = {0x01, 0x97};
	const uint8_t PWM0[2] = {0x02, 0x80};
	const uint8_t PSC1[2] = {0x03, 0x0F};
	const uint8_t PWM1[2] = {0x04, 0x80};
	const uint8_t LS0  = 0x05;

	bool set_led_state(uint8_t led0, uint8_t led1, uint8_t led2) {
		// Convert the LED states into an 8-bit value
		uint8_t states = (0 << 6) | (led2 << 4) | (led1 << 2) | led0;
		
		uint8_t data[2];
		data[0] = LS0;
		data[1] = states;
		
		if (write(file, data, 2) != 2){
			ROS_ERROR("write error set led state");
			return false;
		}
		
		return true;
	}

public:
	PCA9533() {
		// Open I2C bus
		file = open(I2C_DEVICE, O_RDWR);
		if (file < 0) {
			std::cerr << "Failed to open I2C device." << std::endl;
			exit(1);
		}

		// Set the I2C address
		if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
			std::cerr << "Failed to set I2C address." << std::endl;
			exit(1);
		}
		
			// Initial configuration
			if (write(file, &PSC0, 2) != 2){
				ROS_ERROR("write error init config PSC0");
			}
			if (write(file, &PWM0, 2) != 2){
				ROS_ERROR("write error init config PWM0");
			}
			if (write(file, &PSC1, 2) != 2){
				ROS_ERROR("write error init config PSC1");
			}
			if (write(file, &PWM1, 2) != 2){
				ROS_ERROR("write error init config PWM1");
			}
		
		set_led_state(0, 0, 0);

	}

	~PCA9533() {
		if(file>=0){
			set_led_state(0, 0, 0);
			close(file);
		}
	}
	
	bool get_state(i2c_controller_service::i2c_controller_service::Response &response){
		
		States state;
		
		if(!read_led_state(state)){
			return false;
		}

		response.value = static_cast<double>(state);
		return true;
	}
	
	bool read_led_state(States &state) {
		uint8_t data;
		
		if (write(file, &LS0, 1) != 1) {
			ROS_ERROR("readLEDState Failed to write to led controller");
			return false;
		}
		
		usleep(100000);  // 100 ms
		
		if (read(file, &data, 1) != 1) {
			ROS_ERROR("Failed to read led controller");
			return false;
		}
		
		// Data from the registers
		//printf("LS0 Register: 0x%02X\n", data);
		
		
		if(data == 0x05){
			//std::cout<<"green and red \n";
			state = Warning;
			return true;
		}
		else if(data == 0x01){
			//std::cout<<"red  \n";
			state = Critical;
			return true;
		}
		else if(data == 0x04){
			//std::cout<<"green \n";
			state = Ready;
			return true;
		}
		else if(data == 0x08){
			//std::cout<<"green flashing \n";
			state = Recording;
			return true;
		}
		else if(data == 0x00){
			//std::cout<<"OFF \n";
			state = Off;
			return true;
		}
		else if(data == 0x0A){
			//std::cout<<"NoFix \n";
			state = NoFix;
			return true;
		}
		else{
			ROS_ERROR("unrecognized led state");
		}
		
		return false;
	}
	
	bool set_led(const std::string &led_mode){
	
		std::cerr<<"led controler - set led : "<<led_mode <<"\n";
		
		if(led_mode == "error"){
			if(!set_led_state(1, 0, 0)){
				return false;
			}
		}
		else if(led_mode == "recording"){
			if(!set_led_state(0, 2, 0)){
				return false;
			}
		}
		else if(led_mode == "ready"){
			if(!set_led_state(0, 1, 0)){
				return false;
			}
		}
		else if(led_mode == "warning"){
			if(!set_led_state(1, 1, 0)){
				return false;
			}
		}
		else if(led_mode == "nofix"){
			if(!set_led_state(2, 2, 0)){
				return false;
			}
		}
		else{
			ROS_ERROR_STREAM("led controller invalid request: " << led_mode);
			return false;
		}
		
		return true;
	}

};
