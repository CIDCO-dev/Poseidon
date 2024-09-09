#include "ros/ros.h"
#include "led_service/set_led_mode.h"
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


class LEDController {
private:
	const int I2C_ADDR = 0x62;
	const char *I2C_DEVICE = "/dev/i2c-1"; 
	int file;
	
	// PCA9533 Registers
	const uint8_t PSC0[2] = {0x01, 0x97};
	const uint8_t PWM0[2] = {0x02, 0x80};
	const uint8_t PSC1[2] = {0x03, 0x0F};
	const uint8_t PWM1[2] = {0x04, 0x80};
	const uint8_t LS0  = 0x05;
	
	ros::NodeHandle n;
	ros::ServiceServer ledService;

	bool setLEDStates(uint8_t led0, uint8_t led1, uint8_t led2) {
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
	LEDController() {
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
		
		ledService = n.advertiseService("set_led", &LEDController::set_led, this);
		
		setLEDStates(0, 0, 0);

	}

	~LEDController() {
		close(file);
	}
	
	uint8_t readLEDState() {
		uint8_t data;
		
		if (write(file, &LS0, 1) != 1) {
			ROS_ERROR("readLEDState Failed to write to led controller");
		}
		
		usleep(100000);  // 100 ms
		
		if (read(file, &data, 1) != 1) {
			ROS_ERROR("Failed to read led controller");
		}
		
		// Data from the registers
		//printf("LS0 Register: 0x%02X\n", data);
		
		return data;
	}

	void errorON() {
		setLEDStates(1, 0, 0);
	}

	void setRecording() {
		setLEDStates(0, 2, 0);
	}
	
	void setReady(){
		setLEDStates(0,1,0);
	}
	
	bool errorIsON(){
		
		uint8_t ledState = readLEDState();
		ledState = ledState & 0x01;
		if(ledState == 1){
			return true;
		}
		return false;
	}
	
	bool set_led(led_service::set_led_mode::Request &req, led_service::set_led_mode::set_led_mode::Response &res){
		
		if(req.mode == "error"){
			errorON();
		}
		else if(req.mode == "recording"){
			if(!errorIsON()){
				setRecording();
			}
		}
		else if(req.mode == "ready"){
			if(!errorIsON()){
				setReady();
			}
		}
		else{
			ROS_ERROR_STREAM("led controller invalid request: " << req.mode);
		}
		
		return true;
	}

};
