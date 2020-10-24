#ifndef gnss_zed_f9p
#define gnss_zed_f9p

#include "ros/ros.h"


#include <fstream>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <ctime> 
#include <string>
#include <chrono>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>


#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()


#pragma pack(1)
typedef struct {
	char     magic[2];
	uint8_t  msgClass;
	uint8_t  id;
	uint16_t length; //little endian

	uint8_t  classId;
	uint8_t  messageId;

	uint8_t  ck_a;
	uint8_t  ck_b;
} ubx_ack;
#pragma pack()

#pragma pack(1)
typedef struct{
	char     magic[2];
	uint8_t  msgClass;
	uint8_t  id;
	uint16_t length; //little-endian

	uint8_t  version;
	uint8_t  layers;
	uint8_t  reserved[2];
	uint32_t key; //little-endian
	uint8_t  value;

        uint8_t  ck_a;
        uint8_t  ck_b;
} ubx_config;
#pragma pack()

#pragma pack(1)
typedef struct {
        char     magic[2];
        uint8_t  msgClass;
        uint8_t  id;
        uint16_t length; //little endian

        uint16_t navBbrMask; //little endian
        uint8_t  resetMode;
	uint8_t  reserved;

        uint8_t  ck_a;
        uint8_t  ck_b;
} ubx_rst;
#pragma pack()


class ZEDF9P{
	private:
	 	std::string outputFolder;
		std::string serialport;

		std_msgs::String result;
		std_msgs::String ubx_msg_str;
		std_msgs::String ubx_read;

		bool bootstrappedGnssTime = false;

                ros::NodeHandle n;

                ros::Subscriber gnssSubscriber;

	public:

		ZEDF9P(std::string & outputFolder, std::string & serialport): outputFolder(outputFolder), serialport(serialport){
			gnssSubscriber = n.subscribe("fix", 1000, &ZEDF9P::gnssCallback,this);
		}

		std::string datetime(){
    			time_t rawtime;
    			struct tm * timeinfo;
	    		char buffer[80];

    			time (&rawtime);
    			timeinfo = localtime(&rawtime);

   	 		strftime(buffer,80,"%Y.%m.%d_%H%M%S",timeinfo);
    			return std::string(buffer);
		}

		void gnssCallback(const sensor_msgs::NavSatFix& fix){
			if(!bootstrappedGnssTime && fix.status.status >= 0){
				bootstrappedGnssTime = true;
			}
		}

		void run(){
			//serial port opening
			int serial_port = open(serialport.c_str(), O_RDWR);// | O_NOCTTY);
			struct termios tty;
			memset(&tty, 0, sizeof tty);
			//serialport config
			if(tcgetattr(serial_port, &tty) != 0) {
    				printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
			}

			tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
 			tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
			tty.c_cflag |= CS8; // 8 bits per byte (most common)
			tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
			tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

			tty.c_lflag &= ~ICANON;
			tty.c_lflag &= ~ECHO; // Disable echo
			tty.c_lflag &= ~ECHOE; // Disable erasure
			tty.c_lflag &= ~ECHONL; // Disable new-line echo
			tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
			tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
			tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

			tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
			tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
			// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
			// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

			tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
			tty.c_cc[VMIN] = 0;

			// Set in/out baud rate to be 9600
			cfsetispeed(&tty, B460800);
			cfsetospeed(&tty, B460800);

			if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
				printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
			}

			//Init buffer
			int size = 1;
			char read_buf [size];

			//Wait for GNSS fix to init system time
			while(!bootstrappedGnssTime){
				sleep(1);
				ros::spinOnce();
			}

			//create and open file
			std::ofstream file;
			std::string outputFilename = outputFolder + std::string(datetime()) + std::string(".ubx");
			file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

			if(file) {
		                while(ros::ok()){ //read serial port and save in the file
	       		                int n = read(serial_port, &read_buf, size);
					file.write(read_buf, size);
	                	}

				//cleanup
		                close(serial_port);
	        	        file.close();
			}
			else{
				ROS_INFO("Cannot open UBX file\n");
				exit(1);
			}
		}

};


#endif
