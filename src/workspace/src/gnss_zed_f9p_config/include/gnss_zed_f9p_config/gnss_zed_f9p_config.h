#ifndef gnss_zed_f9p_config
#define gnss_zed_f9p_config

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
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

string fnameout="123";
string path="123";

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

	public:

	ZEDF9P(std::string & serialport): serialport(serialport){

	}

	std::string datetime(){
    		time_t rawtime;
    		struct tm * timeinfo;
    		char buffer[80];

    		time (&rawtime);
    		timeinfo = localtime(&rawtime);

    		strftime(buffer,80,"%Y%m%d%H%M%S",timeinfo);
    		return std::string(buffer);
	}

	void checksum(uint8_t * packet,uint8_t size,uint8_t * ck_a,uint8_t * ck_b){
                for(unsigned char i=0 ; i < 13 ; i++){
                        *ck_a += packet[i] ;
                        *ck_b += *ck_a;
                }
	}

	void ubx_cfg(int serial_port, unsigned long ubx_add, char ubx_val, char ubx_layer){
		ubx_config cfg;
		ubx_ack    ack;		

		cfg.magic[0]	= 0xB5;
		cfg.magic[1]	= 0x62;
		cfg.msgClass 	= 0x06;
		cfg.id       	= 0x8A;
		cfg.length   	= 0x09;
		cfg.version  	= 0x00;
		cfg.layers   	= ubx_layer;
		cfg.reserved[0] = 0x00;
		cfg.reserved[1] = 0x00;
		cfg.key 	= ubx_add;
		cfg.value 	= ubx_val;
		cfg.ck_a	= 0;
		cfg.ck_b	= 0;

		checksum(&cfg.msgClass,sizeof(cfg)-4,&cfg.ck_a,&cfg.ck_b); //size - magic - checksum

		int nBytes = write(serial_port, &cfg, sizeof(ubx_config));
		if(nBytes != sizeof(ubx_config)){
       	                 ROS_INFO("Error while writing UBX configuration to serial port: %d bytes written",nBytes);
		        exit(1);
                }

		//ROS_INFO("Write done\n");

		nBytes = read(serial_port,&ack,sizeof(ubx_ack));

		//ROS_INFO("Read done");

		if(nBytes != sizeof(ubx_ack)){
			if (cfg.layers == 0x01) ROS_INFO("RAM: Error while reading CFG_SETVAL response from serial port: %d bytes read",nBytes);
			if (cfg.layers == 0x02) ROS_INFO("BBR: Error while reading CFG_SETVAL response from serial port: %d bytes read",nBytes);
			if (cfg.layers == 0x04) ROS_INFO("FLA: Error while reading CFG_SETVAL response from serial port: %d bytes read",nBytes);
			exit(1);
		}

		if (cfg.layers == 0x01) ROS_INFO("RAM: %08X %s",ubx_add,ack.id==1?"ACK":"NACK");
		if (cfg.layers == 0x02) ROS_INFO("BBR: %08X %s",ubx_add,ack.id==1?"ACK":"NACK");
		if (cfg.layers == 0x04) ROS_INFO("FLA: %08X %s",ubx_add,ack.id==1?"ACK":"NACK");
		usleep(10000);
       	   	}
		

	void run(){
		ROS_INFO("Initializing GPS configuration");

		int serial_port = open(serialport.c_str(), O_RDWR);// | O_NOCTTY);
		
		struct termios tty;
		memset(&tty, 0, sizeof tty);

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
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);

		if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
			printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		
		}




		char layer;
		//Init USB as UBX
		for (int i = 1; i < 4; i++){
			if (i == 1) {layer = 0x01;}
			if (i == 2) {layer = 0x02;}
			if (i == 3) {layer = 0x04;}

			ubx_cfg(serial_port, 0x209100a9, 0, layer); //CFG-MSGOUT-NMEA_ID_DTM
			ubx_cfg(serial_port, 0x209100e0, 0, layer); //CFG-MSGOUT-NMEA_ID_GBS
			ubx_cfg(serial_port, 0x209100bd, 0, layer); //CFG-MSGOUT-NMEA_ID_GGA
			ubx_cfg(serial_port, 0x209100cc, 0, layer); //CFG-MSGOUT-NMEA_ID_GLL
			ubx_cfg(serial_port, 0x209100b8, 0, layer); //CFG-MSGOUT-NMEA_ID_GNS
			ubx_cfg(serial_port, 0x209100d1, 0, layer); //CFG-MSGOUT-NMEA_ID_GRS
			ubx_cfg(serial_port, 0x209100C2, 0, layer); //CFG-MSGOUT-NMEA_ID_GSA
			ubx_cfg(serial_port, 0x209100d6, 0, layer); //CFG-MSGOUT-NMEA_ID_GST
			ubx_cfg(serial_port, 0x209100C7, 0, layer); //CFG-MSGOUT-NMEA_ID_GSV
			ubx_cfg(serial_port, 0x209100ae, 0, layer); //CFG-MSGOUT-NMEA_ID_RMC
			ubx_cfg(serial_port, 0x209100eA, 0, layer); //CFG-MSGOUT-NMEA_ID_VLW
			ubx_cfg(serial_port, 0x209100b3, 0, layer); //CFG-MSGOUT-NMEA_ID_VTG
			ubx_cfg(serial_port, 0x209100db, 0, layer); //CFG-MSGOUT-NMEA_ID_ZDA
			ubx_cfg(serial_port, 0x209100ef, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYP
			ubx_cfg(serial_port, 0x209100f4, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYS
			ubx_cfg(serial_port, 0x209100f9, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYT
			ubx_cfg(serial_port, 0x209102c0, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1005
			ubx_cfg(serial_port, 0x20910361, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1074
			ubx_cfg(serial_port, 0x209102cf, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1077
			ubx_cfg(serial_port, 0x20910366, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1084
			ubx_cfg(serial_port, 0x209102d4, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1087
			ubx_cfg(serial_port, 0x2091036b, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1094
			ubx_cfg(serial_port, 0x2091031b, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1097
			ubx_cfg(serial_port, 0x20910370, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1124
			ubx_cfg(serial_port, 0x209102d9, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1127
			ubx_cfg(serial_port, 0x20910306, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1130
			ubx_cfg(serial_port, 0x20910301, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_0
			ubx_cfg(serial_port, 0x20910384, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_1
			ubx_cfg(serial_port, 0x2091025c, 0, layer); //CFG-MSGOUT-UBX_LOG_INFO
			ubx_cfg(serial_port, 0x20910352, 0, layer); //CFG-MSGOUT-UBX_MON_COMMS
			ubx_cfg(serial_port, 0x209101bc, 0, layer); //CFG-MSGOUT-UBX_MON_HW2
			ubx_cfg(serial_port, 0x20910357, 0, layer); //CFG-MSGOUT-UBX_MON_HW3
			ubx_cfg(serial_port, 0x209101b7, 0, layer); //CFG-MSGOUT-UBX_MON_HW
			ubx_cfg(serial_port, 0x209101a8, 0, layer); //CFG-MSGOUT-UBX_MON_IO
			ubx_cfg(serial_port, 0x20910199, 0, layer); //CFG-MSGOUT-UBX_MON_MSGPP
			ubx_cfg(serial_port, 0x2091035c, 0, layer); //CFG-MSGOUT-UBX_MON_RF
			ubx_cfg(serial_port, 0x209101a3, 0, layer); //CFG-MSGOUT-UBX_MON_RXBUF
			ubx_cfg(serial_port, 0x2091018a, 0, layer); //CFG-MSGOUT-UBX_MON_RXR
			ubx_cfg(serial_port, 0x2091019e, 0, layer); //CFG-MSGOUT-UBX_MON_TXBUF
			ubx_cfg(serial_port, 0x20910068, 0, layer); //CFG-MSGOUT-UBX_NAV_CLOCK
			ubx_cfg(serial_port, 0x2091003b, 0, layer); //CFG-MSGOUT-UBX_NAV_DOP
			ubx_cfg(serial_port, 0x20910162, 0, layer); //CFG-MSGOUT-UBX_NAV_EOE
			ubx_cfg(serial_port, 0x209100a4, 0, layer); //CFG-MSGOUT-UBX_NAV_GEOFENCE
			ubx_cfg(serial_port, 0x20910031, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSECEF
			ubx_cfg(serial_port, 0x20910036, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSLLH
			ubx_cfg(serial_port, 0x20910081, 0, layer); //CFG-MSGOUT-UBX_NAV_ODO
			ubx_cfg(serial_port, 0x20910013, 0, layer); //CFG-MSGOUT-UBX_NAV_ORB
			ubx_cfg(serial_port, 0x20910027, 0, layer); //CFG-MSGOUT-UBX_NAV_POSECEF
			ubx_cfg(serial_port, 0x2091002c, 1, layer); //CFG-MSGOUT-UBX_NAV_POSLLH  <---
			ubx_cfg(serial_port, 0x20910009, 0, layer); //CFG-MSGOUT-UBX_NAV_PVT
			ubx_cfg(serial_port, 0x20910090, 0, layer); //CFG-MSGOUT-UBX_NAV_RELPOSNED
			ubx_cfg(serial_port, 0x20910018, 0, layer); //CFG-MSGOUT-UBX_NAV_SAT
			ubx_cfg(serial_port, 0x20910348, 0, layer); //CFG-MSGOUT-UBX_NAV_SIG
			ubx_cfg(serial_port, 0x2091001d, 0, layer); //CFG-MSGOUT-UBX_NAV_STATUS
			ubx_cfg(serial_port, 0x2091008b, 0, layer); //CFG-MSGOUT-UBX_NAV_SVIN
			ubx_cfg(serial_port, 0x20910054, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEBDS
			ubx_cfg(serial_port, 0x20910059, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGAL
			ubx_cfg(serial_port, 0x2091004f, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGLO
			ubx_cfg(serial_port, 0x2091004a, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGPS
			ubx_cfg(serial_port, 0x20910063, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMELS
			ubx_cfg(serial_port, 0x2091005e, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEUTC
			ubx_cfg(serial_port, 0x20910040, 0, layer); //CFG-MSGOUT-UBX_NAV_VELECEF
			ubx_cfg(serial_port, 0x20910045, 0, layer); //CFG-MSGOUT-UBX_NAV_VELNED
			ubx_cfg(serial_port, 0x20910207, 0, layer); //CFG-MSGOUT-UBX_RXM_MEASX
			ubx_cfg(serial_port, 0x209102A7, 1, layer); //CFG-MSGOUT-UBX_RXM_RAWX  <---
			ubx_cfg(serial_port, 0x20910261, 0, layer); //CFG-MSGOUT-UBX_RXM_RLM
			ubx_cfg(serial_port, 0x2091026b, 0, layer); //CFG-MSGOUT-UBX_RXM_RTMC
			ubx_cfg(serial_port, 0x20910234, 1, layer); //CFG-MSGOUT-UBX_RXM_SFRBX  <---
			ubx_cfg(serial_port, 0x2091017b, 0, layer); //CFG-MSGOUT-UBX_TIM_TM2
			ubx_cfg(serial_port, 0x20910180, 0, layer); //CFG-MSGOUT-UBX_TIM_TP
			ubx_cfg(serial_port, 0x20910095, 0, layer); //CFG-MSGOUT-UBX_TIM_VRFY
	
			//usleep(2000000);
			//Init UART1 as NMEA 0183
			ubx_cfg(serial_port, 0x209100a7, 0, layer); //CFG-MSGOUT-NMEA_ID_DTM
			ubx_cfg(serial_port, 0x209100DE, 0, layer); //CFG-MSGOUT-NMEA_ID_GBS
			ubx_cfg(serial_port, 0x209100bB, 1, layer); //CFG-MSGOUT-NMEA_ID_GGA
			ubx_cfg(serial_port, 0x209100cA, 1, layer); //CFG-MSGOUT-NMEA_ID_GLL
			ubx_cfg(serial_port, 0x209100b6, 0, layer); //CFG-MSGOUT-NMEA_ID_GNS
			ubx_cfg(serial_port, 0x209100CF, 0, layer); //CFG-MSGOUT-NMEA_ID_GRS
			ubx_cfg(serial_port, 0x209100C0, 1, layer); //CFG-MSGOUT-NMEA_ID_GSA
			ubx_cfg(serial_port, 0x209100d4, 0, layer); //CFG-MSGOUT-NMEA_ID_GST
			ubx_cfg(serial_port, 0x209100C5, 1, layer); //CFG-MSGOUT-NMEA_ID_GSV
			ubx_cfg(serial_port, 0x209100aC, 1, layer); //CFG-MSGOUT-NMEA_ID_RMC
			ubx_cfg(serial_port, 0x209100e8, 0, layer); //CFG-MSGOUT-NMEA_ID_VLW
			ubx_cfg(serial_port, 0x209100b1, 1, layer); //CFG-MSGOUT-NMEA_ID_VTG
			ubx_cfg(serial_port, 0x209100d9, 0, layer); //CFG-MSGOUT-NMEA_ID_ZDA
			ubx_cfg(serial_port, 0x209100eD, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYP
			ubx_cfg(serial_port, 0x209100f2, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYS
			ubx_cfg(serial_port, 0x209100f7, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYT
			ubx_cfg(serial_port, 0x209102BE, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1005
			ubx_cfg(serial_port, 0x2091035F, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1074
			ubx_cfg(serial_port, 0x209102cD, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1077
			ubx_cfg(serial_port, 0x20910364, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1084
			ubx_cfg(serial_port, 0x209102d2, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1087
			ubx_cfg(serial_port, 0x20910369, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1094
			ubx_cfg(serial_port, 0x20910319, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1097
			ubx_cfg(serial_port, 0x2091036E, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1124
			ubx_cfg(serial_port, 0x209102d7, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1127
			ubx_cfg(serial_port, 0x20910304, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1130
			ubx_cfg(serial_port, 0x209102FF, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_0
			ubx_cfg(serial_port, 0x20910382, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_1
			ubx_cfg(serial_port, 0x2091025A, 0, layer); //CFG-MSGOUT-UBX_LOG_INFO
			ubx_cfg(serial_port, 0x20910350, 0, layer); //CFG-MSGOUT-UBX_MON_COMMS
			ubx_cfg(serial_port, 0x209101bA, 0, layer); //CFG-MSGOUT-UBX_MON_HW2
			ubx_cfg(serial_port, 0x20910355, 0, layer); //CFG-MSGOUT-UBX_MON_HW3
			ubx_cfg(serial_port, 0x209101b7, 0, layer); //CFG-MSGOUT-UBX_MON_HW
			ubx_cfg(serial_port, 0x209101a6, 0, layer); //CFG-MSGOUT-UBX_MON_IO
			ubx_cfg(serial_port, 0x20910197, 0, layer); //CFG-MSGOUT-UBX_MON_MSGPP
			ubx_cfg(serial_port, 0x2091035A, 0, layer); //CFG-MSGOUT-UBX_MON_RF
			ubx_cfg(serial_port, 0x209101a1, 0, layer); //CFG-MSGOUT-UBX_MON_RXBUF
			ubx_cfg(serial_port, 0x20910188, 0, layer); //CFG-MSGOUT-UBX_MON_RXR
			ubx_cfg(serial_port, 0x2091019C, 0, layer); //CFG-MSGOUT-UBX_MON_TXBUF
			ubx_cfg(serial_port, 0x20910066, 0, layer); //CFG-MSGOUT-UBX_NAV_CLOCK
			ubx_cfg(serial_port, 0x20910039, 0, layer); //CFG-MSGOUT-UBX_NAV_DOP
			ubx_cfg(serial_port, 0x20910160, 0, layer); //CFG-MSGOUT-UBX_NAV_EOE
			ubx_cfg(serial_port, 0x209100a2, 0, layer); //CFG-MSGOUT-UBX_NAV_GEOFENCE
			ubx_cfg(serial_port, 0x2091002F, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSECEF
			ubx_cfg(serial_port, 0x20910034, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSLLH
			ubx_cfg(serial_port, 0x2091007F, 0, layer); //CFG-MSGOUT-UBX_NAV_ODO
			ubx_cfg(serial_port, 0x20910011, 0, layer); //CFG-MSGOUT-UBX_NAV_ORB
			ubx_cfg(serial_port, 0x20910025, 0, layer); //CFG-MSGOUT-UBX_NAV_POSECEF
			ubx_cfg(serial_port, 0x2091002A, 0, layer); //CFG-MSGOUT-UBX_NAV_POSLLH
			ubx_cfg(serial_port, 0x20910007, 0, layer); //CFG-MSGOUT-UBX_NAV_PVT
			ubx_cfg(serial_port, 0x2091008E, 0, layer); //CFG-MSGOUT-UBX_NAV_RELPOSNED
			ubx_cfg(serial_port, 0x20910016, 0, layer); //CFG-MSGOUT-UBX_NAV_SAT
			ubx_cfg(serial_port, 0x20910346, 0, layer); //CFG-MSGOUT-UBX_NAV_SIG
			ubx_cfg(serial_port, 0x2091001B, 0, layer); //CFG-MSGOUT-UBX_NAV_STATUS
			ubx_cfg(serial_port, 0x20910089, 0, layer); //CFG-MSGOUT-UBX_NAV_SVIN
			ubx_cfg(serial_port, 0x20910052, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEBDS
			ubx_cfg(serial_port, 0x20910057, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGAL
			ubx_cfg(serial_port, 0x2091004D, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGLO
			ubx_cfg(serial_port, 0x20910048, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGPS
			ubx_cfg(serial_port, 0x20910061, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMELS
			ubx_cfg(serial_port, 0x2091005C, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEUTC
			ubx_cfg(serial_port, 0x2091003E, 0, layer); //CFG-MSGOUT-UBX_NAV_VELECEF
			ubx_cfg(serial_port, 0x20910043, 0, layer); //CFG-MSGOUT-UBX_NAV_VELNED
			ubx_cfg(serial_port, 0x20910205, 0, layer); //CFG-MSGOUT-UBX_RXM_MEASX
			ubx_cfg(serial_port, 0x209102A5, 0, layer); //CFG-MSGOUT-UBX_RXM_RAWX
			ubx_cfg(serial_port, 0x2091025F, 0, layer); //CFG-MSGOUT-UBX_RXM_RLM
			ubx_cfg(serial_port, 0x20910269, 0, layer); //CFG-MSGOUT-UBX_RXM_RTMC
			ubx_cfg(serial_port, 0x20910232, 0, layer); //CFG-MSGOUT-UBX_RXM_SFRBX
			ubx_cfg(serial_port, 0x20910179, 0, layer); //CFG-MSGOUT-UBX_TIM_TM2
			ubx_cfg(serial_port, 0x2091017E, 0, layer); //CFG-MSGOUT-UBX_TIM_TP
			ubx_cfg(serial_port, 0x20910093, 0, layer); //CFG-MSGOUT-UBX_TIM_VRFY
			
			//Init UART2 as NMEA 0183
			ubx_cfg(serial_port, 0x209100a8, 0, layer); //CFG-MSGOUT-NMEA_ID_DTM
			ubx_cfg(serial_port, 0x209100DF, 0, layer); //CFG-MSGOUT-NMEA_ID_GBS
			ubx_cfg(serial_port, 0x209100bC, 1, layer); //CFG-MSGOUT-NMEA_ID_GGA
			ubx_cfg(serial_port, 0x209100cB, 1, layer); //CFG-MSGOUT-NMEA_ID_GLL
			ubx_cfg(serial_port, 0x209100b7, 0, layer); //CFG-MSGOUT-NMEA_ID_GNS
			ubx_cfg(serial_port, 0x209100D0, 0, layer); //CFG-MSGOUT-NMEA_ID_GRS
			ubx_cfg(serial_port, 0x209100C1, 1, layer); //CFG-MSGOUT-NMEA_ID_GSA
			ubx_cfg(serial_port, 0x209100d5, 0, layer); //CFG-MSGOUT-NMEA_ID_GST
			ubx_cfg(serial_port, 0x209100C6, 1, layer); //CFG-MSGOUT-NMEA_ID_GSV
			ubx_cfg(serial_port, 0x209100aD, 1, layer); //CFG-MSGOUT-NMEA_ID_RMC
			ubx_cfg(serial_port, 0x209100e9, 0, layer); //CFG-MSGOUT-NMEA_ID_VLW
			ubx_cfg(serial_port, 0x209100b2, 1, layer); //CFG-MSGOUT-NMEA_ID_VTG
			ubx_cfg(serial_port, 0x209100dA, 0, layer); //CFG-MSGOUT-NMEA_ID_ZDA
			ubx_cfg(serial_port, 0x209100eE, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYP
			ubx_cfg(serial_port, 0x209100f3, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYS
			ubx_cfg(serial_port, 0x209100f8, 0, layer); //CFG-MSGOUT-PUBX_ID_POLYT
			ubx_cfg(serial_port, 0x209102BF, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1005
			ubx_cfg(serial_port, 0x20910360, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1074
			ubx_cfg(serial_port, 0x209102cE, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1077
			ubx_cfg(serial_port, 0x20910365, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1084
			ubx_cfg(serial_port, 0x209102d3, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1087
			ubx_cfg(serial_port, 0x2091036A, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1094
			ubx_cfg(serial_port, 0x2091031A, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1097
			ubx_cfg(serial_port, 0x2091036F, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1124
			ubx_cfg(serial_port, 0x209102d8, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1127
			ubx_cfg(serial_port, 0x20910305, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE1130
			ubx_cfg(serial_port, 0x20910300, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_0
			ubx_cfg(serial_port, 0x20910383, 0, layer); //CFG-MSGOUT-RTCM_3X_TYPE4072_1
			ubx_cfg(serial_port, 0x2091025B, 0, layer); //CFG-MSGOUT-UBX_LOG_INFO
			ubx_cfg(serial_port, 0x20910351, 0, layer); //CFG-MSGOUT-UBX_MON_COMMS
			ubx_cfg(serial_port, 0x209101bB, 0, layer); //CFG-MSGOUT-UBX_MON_HW2
			ubx_cfg(serial_port, 0x20910356, 0, layer); //CFG-MSGOUT-UBX_MON_HW3
			ubx_cfg(serial_port, 0x209101b8, 0, layer); //CFG-MSGOUT-UBX_MON_HW
			ubx_cfg(serial_port, 0x209101a7, 0, layer); //CFG-MSGOUT-UBX_MON_IO
			ubx_cfg(serial_port, 0x20910198, 0, layer); //CFG-MSGOUT-UBX_MON_MSGPP
			ubx_cfg(serial_port, 0x2091035B, 0, layer); //CFG-MSGOUT-UBX_MON_RF
			ubx_cfg(serial_port, 0x209101a2, 0, layer); //CFG-MSGOUT-UBX_MON_RXBUF
			ubx_cfg(serial_port, 0x20910189, 0, layer); //CFG-MSGOUT-UBX_MON_RXR
			ubx_cfg(serial_port, 0x2091019D, 0, layer); //CFG-MSGOUT-UBX_MON_TXBUF
			ubx_cfg(serial_port, 0x20910067, 0, layer); //CFG-MSGOUT-UBX_NAV_CLOCK
			ubx_cfg(serial_port, 0x2091003A, 0, layer); //CFG-MSGOUT-UBX_NAV_DOP
			ubx_cfg(serial_port, 0x20910161, 0, layer); //CFG-MSGOUT-UBX_NAV_EOE
			ubx_cfg(serial_port, 0x209100a3, 0, layer); //CFG-MSGOUT-UBX_NAV_GEOFENCE
			ubx_cfg(serial_port, 0x20910030, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSECEF
			ubx_cfg(serial_port, 0x20910035, 0, layer); //CFG-MSGOUT-UBX_NAV_HPPOSLLH
			ubx_cfg(serial_port, 0x20910080, 0, layer); //CFG-MSGOUT-UBX_NAV_ODO
			ubx_cfg(serial_port, 0x20910012, 0, layer); //CFG-MSGOUT-UBX_NAV_ORB
			ubx_cfg(serial_port, 0x20910026, 0, layer); //CFG-MSGOUT-UBX_NAV_POSECEF
			ubx_cfg(serial_port, 0x2091002B, 0, layer); //CFG-MSGOUT-UBX_NAV_POSLLH
			ubx_cfg(serial_port, 0x20910008, 0, layer); //CFG-MSGOUT-UBX_NAV_PVT
			ubx_cfg(serial_port, 0x2091008F, 0, layer); //CFG-MSGOUT-UBX_NAV_RELPOSNED
			ubx_cfg(serial_port, 0x20910017, 0, layer); //CFG-MSGOUT-UBX_NAV_SAT
			ubx_cfg(serial_port, 0x20910347, 0, layer); //CFG-MSGOUT-UBX_NAV_SIG
			ubx_cfg(serial_port, 0x2091001C, 0, layer); //CFG-MSGOUT-UBX_NAV_STATUS
			ubx_cfg(serial_port, 0x2091008A, 0, layer); //CFG-MSGOUT-UBX_NAV_SVIN
			ubx_cfg(serial_port, 0x20910053, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEBDS
			ubx_cfg(serial_port, 0x20910058, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGAL
			ubx_cfg(serial_port, 0x2091004E, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGLO
			ubx_cfg(serial_port, 0x20910049, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEGPS
			ubx_cfg(serial_port, 0x20910062, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMELS
			ubx_cfg(serial_port, 0x2091005D, 0, layer); //CFG-MSGOUT-UBX_NAV_TIMEUTC
			ubx_cfg(serial_port, 0x2091003F, 0, layer); //CFG-MSGOUT-UBX_NAV_VELECEF
			ubx_cfg(serial_port, 0x20910044, 0, layer); //CFG-MSGOUT-UBX_NAV_VELNED
			ubx_cfg(serial_port, 0x20910206, 0, layer); //CFG-MSGOUT-UBX_RXM_MEASX
			ubx_cfg(serial_port, 0x209102A6, 0, layer); //CFG-MSGOUT-UBX_RXM_RAWX
			ubx_cfg(serial_port, 0x20910260, 0, layer); //CFG-MSGOUT-UBX_RXM_RLM
			ubx_cfg(serial_port, 0x2091026A, 0, layer); //CFG-MSGOUT-UBX_RXM_RTMC
			ubx_cfg(serial_port, 0x20910233, 0, layer); //CFG-MSGOUT-UBX_RXM_SFRBX
			ubx_cfg(serial_port, 0x2091017A, 0, layer); //CFG-MSGOUT-UBX_TIM_TM2
			ubx_cfg(serial_port, 0x2091017F, 0, layer); //CFG-MSGOUT-UBX_TIM_TP
			ubx_cfg(serial_port, 0x20910094, 0, layer); //CFG-MSGOUT-UBX_TIM_VRFY
			
			//usleep(2000000);
		}
		close(serial_port);
		ROS_INFO("GPS configuration done");
                //while(ros::ok()){
                //}
	}
};


#endif
