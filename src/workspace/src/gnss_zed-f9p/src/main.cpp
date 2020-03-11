#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"

#include <stdio.h>
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


class ZEDF9P{
	private:
	std::string outputFolder;
	std::string serialport;
	std_msgs::String result;
	std_msgs::String ubx_msg_str;
	//uint8_t *buffer;
	std_msgs::String ubx_read;
	
	public:

	ZEDF9P(char * outputFolder,  char * serialport): outputFolder(outputFolder), serialport(serialport){
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
	void ubx_cfg(unsigned long ubx_add, char ubx_val){
		unsigned char ubx_header1 = 181;
		unsigned char ubx_header2 = 98;
		unsigned char ubx_class = 6;
		unsigned char ubx_id = 0x8A;
		unsigned int ubx_chksum_A = 0x00;
		unsigned int ubx_chksum_B = 0x00;
		unsigned char ubx_version = 0x01;
		unsigned char ubx_layer = 0x01;
		unsigned char ubx_reserved = 0x00;
		
		unsigned char ubx_buffer[13];
		char ubx_msg[17];
		
		

	
		unsigned char ubx_lenght = 0x09;

		ubx_buffer[0] = ubx_class;
		ubx_buffer[1] = ubx_id;
		ubx_buffer[2] = ubx_lenght;
		ubx_buffer[3] = 0x00;
		ubx_buffer[4] = 0x00;
		ubx_buffer[5] = ubx_layer;
		ubx_buffer[6] = 0x00;
		ubx_buffer[7] = 0x00;
		ubx_buffer[8] = (ubx_add & 0x000000FF) ;
		ubx_buffer[9] = ((ubx_add & 0x0000FF00) >> 8);
		ubx_buffer[10] = ((ubx_add & 0x00FF0000) >> 16);
		ubx_buffer[11] = ((ubx_add & 0xFF000000) >> 24);
		ubx_buffer[12] = ubx_val;



		unsigned char ubx_buffer_lenght = 13;

		for(unsigned char I=0 ; I<ubx_buffer_lenght; I++){
			ubx_chksum_A = (ubx_chksum_A + ubx_buffer[I]) & 0xFF;
			ubx_chksum_B = (ubx_chksum_B + ubx_chksum_A) & 0xFF;
		}
		//ubx_chksum_A = ubx_chksum_A & 0xFF;
		//ubx_chksum_B = ubx_chksum_B & 0xFF;

		ubx_msg[0] = ubx_header1;
		ubx_msg[1] = ubx_header2;
		ubx_msg[2] = ubx_buffer[0];
		ubx_msg[3] = ubx_buffer[1];
		ubx_msg[4] = ubx_buffer[2];
		ubx_msg[5] = ubx_buffer[3];
		ubx_msg[6] = ubx_buffer[4];
		ubx_msg[7] = ubx_buffer[5];
		ubx_msg[8] = ubx_buffer[6];
		ubx_msg[9] = ubx_buffer[7];
		ubx_msg[10] = ubx_buffer[8];
		ubx_msg[11] = ubx_buffer[9];
		ubx_msg[12] = ubx_buffer[10];
		ubx_msg[13] = ubx_buffer[11];
		ubx_msg[14] = ubx_buffer[12];
		ubx_msg[15] = ubx_chksum_A;
		ubx_msg[16] = ubx_chksum_B;
		int serial_port = open(serialport.c_str(), O_RDWR);

		write(serial_port, ubx_msg, 17);
		
		
		ubx_msg[7] = ubx_msg[7]+1;
		ubx_msg[15] = ubx_msg[15]+1;
		ubx_msg[16] = ubx_msg[16]+8;

		write(serial_port, ubx_msg, 17);
		
		ubx_msg[7] = ubx_msg[7]+2;
		ubx_msg[15] = ubx_msg[15]+2;
		ubx_msg[16] = ubx_msg[16]+16;

		write(serial_port, ubx_msg, 17);
		
		close(serial_port);

		}

	void run(){
		cout << "gps config" << endl;
		//écrit la configuration du module gps pour le port usb
		ubx_cfg(0x209100a9, 0); //CFG-MSGOUT-NMEA_ID_DTM
		ubx_cfg(0x209100e0, 0); //CFG-MSGOUT-NMEA_ID_GBS
		ubx_cfg(0x209100bd, 0); //CFG-MSGOUT-NMEA_ID_GGA
		ubx_cfg(0x209100cc, 0); //CFG-MSGOUT-NMEA_ID_GLL
		ubx_cfg(0x209100b8, 0); //CFG-MSGOUT-NMEA_ID_GNS
		ubx_cfg(0x209100d1, 0); //CFG-MSGOUT-NMEA_ID_GRS
		ubx_cfg(0x209100C2, 0); //CFG-MSGOUT-NMEA_ID_GSA
		ubx_cfg(0x209100d6, 0); //CFG-MSGOUT-NMEA_ID_GST
		ubx_cfg(0x209100C7, 0); //CFG-MSGOUT-NMEA_ID_GSV
		ubx_cfg(0x209100ae, 0); //CFG-MSGOUT-NMEA_ID_RMC
		ubx_cfg(0x209100eA, 0); //CFG-MSGOUT-NMEA_ID_VLW
		ubx_cfg(0x209100b3, 0); //CFG-MSGOUT-NMEA_ID_VTG
		ubx_cfg(0x209100db, 0); //CFG-MSGOUT-NMEA_ID_ZDA
		ubx_cfg(0x209100ef, 0); //CFG-MSGOUT-PUBX_ID_POLYP
		ubx_cfg(0x209100f4, 0); //CFG-MSGOUT-PUBX_ID_POLYS
		ubx_cfg(0x209100f9, 0); //CFG-MSGOUT-PUBX_ID_POLYT
		ubx_cfg(0x209102c0, 0); //CFG-MSGOUT-RTCM_3X_TYPE1005
		ubx_cfg(0x20910361, 0); //CFG-MSGOUT-RTCM_3X_TYPE1074
		ubx_cfg(0x209102cf, 0); //CFG-MSGOUT-RTCM_3X_TYPE1077
		ubx_cfg(0x20910366, 0); //CFG-MSGOUT-RTCM_3X_TYPE1084
		ubx_cfg(0x209102d4, 0); //CFG-MSGOUT-RTCM_3X_TYPE1087
		ubx_cfg(0x2091036b, 0); //CFG-MSGOUT-RTCM_3X_TYPE1094
		ubx_cfg(0x2091031b, 0); //CFG-MSGOUT-RTCM_3X_TYPE1097
		ubx_cfg(0x20910370, 0); //CFG-MSGOUT-RTCM_3X_TYPE1124
		ubx_cfg(0x209102d9, 0); //CFG-MSGOUT-RTCM_3X_TYPE1127
		ubx_cfg(0x20910306, 0); //CFG-MSGOUT-RTCM_3X_TYPE1130
		ubx_cfg(0x20910301, 0); //CFG-MSGOUT-RTCM_3X_TYPE4072_0
		ubx_cfg(0x20910384, 0); //CFG-MSGOUT-RTCM_3X_TYPE4072_1
		ubx_cfg(0x2091025c, 0); //CFG-MSGOUT-UBX_LOG_INFO
		ubx_cfg(0x20910352, 0); //CFG-MSGOUT-UBX_MON_COMMS
		ubx_cfg(0x209101bc, 0); //CFG-MSGOUT-UBX_MON_HW2
		ubx_cfg(0x20910357, 0); //CFG-MSGOUT-UBX_MON_HW3
		ubx_cfg(0x209101b7, 0); //CFG-MSGOUT-UBX_MON_HW
		ubx_cfg(0x209101a8, 0); //CFG-MSGOUT-UBX_MON_IO
		ubx_cfg(0x20910199, 0); //CFG-MSGOUT-UBX_MON_MSGPP
		ubx_cfg(0x2091035c, 0); //CFG-MSGOUT-UBX_MON_RF
		ubx_cfg(0x209101a3, 0); //CFG-MSGOUT-UBX_MON_RXBUF
		ubx_cfg(0x2091018a, 0); //CFG-MSGOUT-UBX_MON_RXR
		ubx_cfg(0x2091019e, 0); //CFG-MSGOUT-UBX_MON_TXBUF
		ubx_cfg(0x20910068, 0); //CFG-MSGOUT-UBX_NAV_CLOCK
		ubx_cfg(0x2091003b, 0); //CFG-MSGOUT-UBX_NAV_DOP
		ubx_cfg(0x20910162, 0); //CFG-MSGOUT-UBX_NAV_EOE
		ubx_cfg(0x209100a4, 0); //CFG-MSGOUT-UBX_NAV_GEOFENCE
		ubx_cfg(0x20910031, 0); //CFG-MSGOUT-UBX_NAV_HPPOSECEF
		ubx_cfg(0x20910036, 0); //CFG-MSGOUT-UBX_NAV_HPPOSLLH
		ubx_cfg(0x20910081, 0); //CFG-MSGOUT-UBX_NAV_ODO
		ubx_cfg(0x20910013, 0); //CFG-MSGOUT-UBX_NAV_ORB
		ubx_cfg(0x20910027, 0); //CFG-MSGOUT-UBX_NAV_POSECEF
		ubx_cfg(0x2091002c, 1); //CFG-MSGOUT-UBX_NAV_POSLLH  <---
		ubx_cfg(0x20910009, 0); //CFG-MSGOUT-UBX_NAV_PVT
		ubx_cfg(0x20910090, 0); //CFG-MSGOUT-UBX_NAV_RELPOSNED
		ubx_cfg(0x20910018, 0); //CFG-MSGOUT-UBX_NAV_SAT
		ubx_cfg(0x20910348, 0); //CFG-MSGOUT-UBX_NAV_SIG
		ubx_cfg(0x2091001d, 0); //CFG-MSGOUT-UBX_NAV_STATUS
		ubx_cfg(0x2091008b, 0); //CFG-MSGOUT-UBX_NAV_SVIN
		ubx_cfg(0x20910054, 0); //CFG-MSGOUT-UBX_NAV_TIMEBDS
		ubx_cfg(0x20910059, 0); //CFG-MSGOUT-UBX_NAV_TIMEGAL
		ubx_cfg(0x2091004f, 0); //CFG-MSGOUT-UBX_NAV_TIMEGLO
		ubx_cfg(0x2091004a, 0); //CFG-MSGOUT-UBX_NAV_TIMEGPS
		ubx_cfg(0x20910063, 0); //CFG-MSGOUT-UBX_NAV_TIMELS
		ubx_cfg(0x2091005e, 0); //CFG-MSGOUT-UBX_NAV_TIMEUTC
		ubx_cfg(0x20910040, 0); //CFG-MSGOUT-UBX_NAV_VELECEF
		ubx_cfg(0x20910045, 0); //CFG-MSGOUT-UBX_NAV_VELNED
		ubx_cfg(0x20910207, 0); //CFG-MSGOUT-UBX_RXM_MEASX
		ubx_cfg(0x209102A7, 1); //CFG-MSGOUT-UBX_RXM_RAWX  <---
		ubx_cfg(0x20910261, 0); //CFG-MSGOUT-UBX_RXM_RLM
		ubx_cfg(0x2091026b, 0); //CFG-MSGOUT-UBX_RXM_RTMC
		ubx_cfg(0x20910234, 1); //CFG-MSGOUT-UBX_RXM_SFRBX  <---
		ubx_cfg(0x2091017b, 0); //CFG-MSGOUT-UBX_TIM_TM2
		ubx_cfg(0x20910180, 0); //CFG-MSGOUT-UBX_TIM_TP
		ubx_cfg(0x20910095, 0); //CFG-MSGOUT-UBX_TIM_VRFY


		//écrit la configuration du module gps pour le port UART1
		ubx_cfg(0x209100a7, 0); //CFG-MSGOUT-NMEA_ID_DTM
		ubx_cfg(0x209100DE, 0); //CFG-MSGOUT-NMEA_ID_GBS
		ubx_cfg(0x209100bB, 1); //CFG-MSGOUT-NMEA_ID_GGA
		ubx_cfg(0x209100cA, 1); //CFG-MSGOUT-NMEA_ID_GLL
		ubx_cfg(0x209100b6, 0); //CFG-MSGOUT-NMEA_ID_GNS
		ubx_cfg(0x209100CF, 0); //CFG-MSGOUT-NMEA_ID_GRS
		ubx_cfg(0x209100C0, 1); //CFG-MSGOUT-NMEA_ID_GSA
		ubx_cfg(0x209100d4, 0); //CFG-MSGOUT-NMEA_ID_GST
		ubx_cfg(0x209100C5, 1); //CFG-MSGOUT-NMEA_ID_GSV
		ubx_cfg(0x209100aC, 1); //CFG-MSGOUT-NMEA_ID_RMC
		ubx_cfg(0x209100e8, 0); //CFG-MSGOUT-NMEA_ID_VLW
		ubx_cfg(0x209100b1, 1); //CFG-MSGOUT-NMEA_ID_VTG
		ubx_cfg(0x209100d9, 0); //CFG-MSGOUT-NMEA_ID_ZDA
		ubx_cfg(0x209100eD, 0); //CFG-MSGOUT-PUBX_ID_POLYP
		ubx_cfg(0x209100f2, 0); //CFG-MSGOUT-PUBX_ID_POLYS
		ubx_cfg(0x209100f7, 0); //CFG-MSGOUT-PUBX_ID_POLYT
		ubx_cfg(0x209102BE, 0); //CFG-MSGOUT-RTCM_3X_TYPE1005
		ubx_cfg(0x2091035F, 0); //CFG-MSGOUT-RTCM_3X_TYPE1074
		ubx_cfg(0x209102cD, 0); //CFG-MSGOUT-RTCM_3X_TYPE1077
		ubx_cfg(0x20910364, 0); //CFG-MSGOUT-RTCM_3X_TYPE1084
		ubx_cfg(0x209102d2, 0); //CFG-MSGOUT-RTCM_3X_TYPE1087
		ubx_cfg(0x20910369, 0); //CFG-MSGOUT-RTCM_3X_TYPE1094
		ubx_cfg(0x20910319, 0); //CFG-MSGOUT-RTCM_3X_TYPE1097
		ubx_cfg(0x2091036E, 0); //CFG-MSGOUT-RTCM_3X_TYPE1124
		ubx_cfg(0x209102d7, 0); //CFG-MSGOUT-RTCM_3X_TYPE1127
		ubx_cfg(0x20910304, 0); //CFG-MSGOUT-RTCM_3X_TYPE1130
		ubx_cfg(0x209102FF, 0); //CFG-MSGOUT-RTCM_3X_TYPE4072_0
		ubx_cfg(0x20910382, 0); //CFG-MSGOUT-RTCM_3X_TYPE4072_1
		ubx_cfg(0x2091025A, 0); //CFG-MSGOUT-UBX_LOG_INFO
		ubx_cfg(0x20910350, 0); //CFG-MSGOUT-UBX_MON_COMMS
		ubx_cfg(0x209101bA, 0); //CFG-MSGOUT-UBX_MON_HW2
		ubx_cfg(0x20910355, 0); //CFG-MSGOUT-UBX_MON_HW3
		ubx_cfg(0x209101b7, 0); //CFG-MSGOUT-UBX_MON_HW
		ubx_cfg(0x209101a6, 0); //CFG-MSGOUT-UBX_MON_IO
		ubx_cfg(0x20910197, 0); //CFG-MSGOUT-UBX_MON_MSGPP
		ubx_cfg(0x2091035A, 0); //CFG-MSGOUT-UBX_MON_RF
		ubx_cfg(0x209101a1, 0); //CFG-MSGOUT-UBX_MON_RXBUF
		ubx_cfg(0x20910188, 0); //CFG-MSGOUT-UBX_MON_RXR
		ubx_cfg(0x2091019C, 0); //CFG-MSGOUT-UBX_MON_TXBUF
		ubx_cfg(0x20910066, 0); //CFG-MSGOUT-UBX_NAV_CLOCK
		ubx_cfg(0x20910039, 0); //CFG-MSGOUT-UBX_NAV_DOP
		ubx_cfg(0x20910160, 0); //CFG-MSGOUT-UBX_NAV_EOE
		ubx_cfg(0x209100a2, 0); //CFG-MSGOUT-UBX_NAV_GEOFENCE
		ubx_cfg(0x2091002F, 0); //CFG-MSGOUT-UBX_NAV_HPPOSECEF
		ubx_cfg(0x20910034, 0); //CFG-MSGOUT-UBX_NAV_HPPOSLLH
		ubx_cfg(0x2091007F, 0); //CFG-MSGOUT-UBX_NAV_ODO
		ubx_cfg(0x20910011, 0); //CFG-MSGOUT-UBX_NAV_ORB
		ubx_cfg(0x20910025, 0); //CFG-MSGOUT-UBX_NAV_POSECEF
		ubx_cfg(0x2091002A, 0); //CFG-MSGOUT-UBX_NAV_POSLLH
		ubx_cfg(0x20910007, 0); //CFG-MSGOUT-UBX_NAV_PVT
		ubx_cfg(0x2091008E, 0); //CFG-MSGOUT-UBX_NAV_RELPOSNED
		ubx_cfg(0x20910016, 0); //CFG-MSGOUT-UBX_NAV_SAT
		ubx_cfg(0x20910346, 0); //CFG-MSGOUT-UBX_NAV_SIG
		ubx_cfg(0x2091001B, 0); //CFG-MSGOUT-UBX_NAV_STATUS
		ubx_cfg(0x20910089, 0); //CFG-MSGOUT-UBX_NAV_SVIN
		ubx_cfg(0x20910052, 0); //CFG-MSGOUT-UBX_NAV_TIMEBDS
		ubx_cfg(0x20910057, 0); //CFG-MSGOUT-UBX_NAV_TIMEGAL
		ubx_cfg(0x2091004D, 0); //CFG-MSGOUT-UBX_NAV_TIMEGLO
		ubx_cfg(0x20910048, 0); //CFG-MSGOUT-UBX_NAV_TIMEGPS
		ubx_cfg(0x20910061, 0); //CFG-MSGOUT-UBX_NAV_TIMELS
		ubx_cfg(0x2091005C, 0); //CFG-MSGOUT-UBX_NAV_TIMEUTC
		ubx_cfg(0x2091003E, 0); //CFG-MSGOUT-UBX_NAV_VELECEF
		ubx_cfg(0x20910043, 0); //CFG-MSGOUT-UBX_NAV_VELNED
		ubx_cfg(0x20910205, 0); //CFG-MSGOUT-UBX_RXM_MEASX
		ubx_cfg(0x209102A5, 0); //CFG-MSGOUT-UBX_RXM_RAWX
		ubx_cfg(0x2091025F, 0); //CFG-MSGOUT-UBX_RXM_RLM
		ubx_cfg(0x20910269, 0); //CFG-MSGOUT-UBX_RXM_RTMC
		ubx_cfg(0x20910232, 0); //CFG-MSGOUT-UBX_RXM_SFRBX
		ubx_cfg(0x20910179, 0); //CFG-MSGOUT-UBX_TIM_TM2
		ubx_cfg(0x2091017E, 0); //CFG-MSGOUT-UBX_TIM_TP
		ubx_cfg(0x20910093, 0); //CFG-MSGOUT-UBX_TIM_VRFY

		cout << "gps loging" << endl;


		//lis le temps si la seconde change on change le nom du fichier
		fnameout = outputFolder + datetime() +".ubx";
		
		char read_buf [256];
		memset(&read_buf, '\0', sizeof(read_buf));
		int serial_port1 = open(serialport.c_str(), O_RDWR);
		int file = open(fnameout.c_str(), O_RDWR | O_CREAT | O_APPEND);
		while(ros::ok()){
			
			
			int n = read(serial_port1, &read_buf, sizeof(read_buf));
			write(file, &read_buf, sizeof(read_buf));

			
		
		}
		close(serial_port1);
		close(file);
	}


};
int main(int argc,char** argv){
  	
	ros::init(argc, argv, "zedf9p");
	std::string addr (argv[1]);
	std::string port (argv[2]);


	if (addr.length()<2){
		std::cout << "nlogger_text, Missing output folder path" << std::endl;
    		return 1;
	}

	if (port.length()<2){
		std::cout << "nlogger_text, Missing serial port" << std::endl;
    		return 1;

	}


	//création du path du log si non présent
	
	//initialiser le module zed-f9p

	ZEDF9P zedf9p(argv[1] ,argv[2]);
	zedf9p.run();
	
}
#endif
