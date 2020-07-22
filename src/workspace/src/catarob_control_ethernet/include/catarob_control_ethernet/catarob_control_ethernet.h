#ifndef catarob_control_ethernet
#define catarob_control_ethernet

#include "ros/ros.h"
#include "catarob_msg/motor.h"
#include "catarob_msg/state.h"




#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

class shell_left_data{
	public:
		unsigned char add;
		unsigned char actual_wpm;
		unsigned char water;
		unsigned char relay_state;
		unsigned char i_bat;
		unsigned char i_bat_ech;
		unsigned char i_max;
		unsigned char i_max_ech;
		unsigned char i_motor;
		unsigned char i_motor_ech;
		unsigned char pwm_rc;
		unsigned char pwm_source;
		unsigned char status;
		unsigned char temperature;
		unsigned char tor_rc;
		unsigned char vbat;
		unsigned char vbat_ech;
		unsigned char vbat2;
		unsigned char vbat2_ech;
		unsigned char v5v;
		unsigned char v5v_ech;
		unsigned char v12v;
		unsigned char v12v_ech;
		unsigned char v24v;
		unsigned char v24v_ech;
};

class shell_right_data{
	public:
		unsigned char add;
		unsigned char actual_wpm;
		unsigned char water;
		unsigned char relay_state;
		unsigned char i_bat;
		unsigned char i_bat_ech;
		unsigned char i_max;
		unsigned char i_max_ech;
		unsigned char i_motor;
		unsigned char i_motor_ech;
		unsigned char pwm_rc;
		unsigned char pwm_source;
		unsigned char status;
		unsigned char temperature;
		unsigned char tor_rc;
		unsigned char vbat;
		unsigned char vbat_ech;
		unsigned char vbat2;
		unsigned char vbat2_ech;
		unsigned char v5v;
		unsigned char v5v_ech;
		unsigned char v12v;
		unsigned char v12v_ech;
		unsigned char v24v;
		unsigned char v24v_ech;
};

using namespace std;

int fdL,fdR;
unsigned short pt_cei012c_tab_ech[30] = {96,124,157,196,241,290,343,398,455,512,567,619,668,712,753,788,820,847,871,892,910,925,938,949,959,967,974,981,986,990};
uint16_t ADCTempVal = 0;
std::mutex mtx;

char low_byte(int x){
	//convert low byte form integer
	return (x & 0x00FF);
}

char hi_byte(int x){
	//convert hi byte from integer
	return (x & 0xFF00) >> 8;
}

int int_from_byte(char hi, char lo){
	//convert 2 bytes in one integer
	return ((hi * 0xFF)+lo);
}


class CATAROB{
	private:
		ros::NodeHandle n;
		ros::Publisher state_L;
		ros::Publisher state_R;

		std::string serverAddress;
		std::string serverPort;


	public:

	CATAROB(char * serverAddress, char * serverPort) : serverAddress(serverAddress),serverPort(serverPort){

		ros::Subscriber motor_L = n.subscribe("motor/left", 1000, &CATAROB::motorLcallback,this);
		ros::Subscriber motor_R = n.subscribe("motor/right", 1000, &CATAROB::motorRcallback,this);
		state_L= n.advertise<catarob_msg::state>("state/left", 1000);
		state_R= n.advertise<catarob_msg::state>("state/right", 1000);
		}

	void motortransmit(char id,char pwm,char imaxl,char imaxh,char relay){ //section a travailler une fois la connection au serveur faite
		unsigned char *WRbuffer;
		//Allocation des buffer de ecriture
    	WRbuffer=(unsigned char*)malloc(15*sizeof(unsigned char));
    	WRbuffer[0]= 0;
    	WRbuffer[1]= pwm;		//pwm low
    	WRbuffer[2]= 0x00;		//pwm high
    	WRbuffer[3]= relay;		//relais low
    	WRbuffer[4]= 0x00;		//relais hi
    	WRbuffer[5]= imaxl;		//imax low
    	WRbuffer[6]= imaxh;		//imax hi
    	WRbuffer[7]= relay;		//relais1 loq
    	WRbuffer[8]= 0x00;		//relais1 hi
    	WRbuffer[9]= relay;		//relais2 low
    	WRbuffer[10]=0x00;		//relais2 hi
    	WRbuffer[11]=relay;		//relais3 low
    	WRbuffer[12]=0x00;		//relais3 hi
    	WRbuffer[13]=51;		//utilisation i2c

  		//création du checksum

    	//écriture des valeur


		free(WRbuffer);
		}

	void read_shell(char id){ //section a travailler une fois la connection au serveur faite
		catarob_msg::state msg1;
		//requete des donnée en fonction du id
		
		//atente de réception des données
		
		//conversion des donnée recu pour les rendre compatible avec le topic
		
		//transmission des donnée sur le topic
		if (id == fdL) state_L.publish(msg1);
		if (id == fdR) state_R.publish(msg1);
		}

	void motorLcallback(const catarob_msg::motor& motor_L)
		{
		//convert toopic.msg to call motortransmit	
		motortransmit(fdL,motor_L.pwm,motor_L.imaxl,motor_L.imaxh,motor_L.relay);
		}

	void motorRcallback(const catarob_msg::motor& motor_R)
		{
		//convert toopic.msg to call motortransmit	
		motortransmit(fdR,motor_R.pwm,motor_R.imaxl,motor_R.imaxh,motor_R.relay);
		}

	void read_left_shell() 
		{
		//call read shell for the left side
		read_shell(fdL);
		}

	void read_right_shell()
		{
		//call read shell for the right side
		read_shell(fdR);
		}


	void run(){
		ros::Rate loop_rate(1);
		while(ros::ok()){
			//read_winch(); //a implémenter plus tard
			//read_spool(); //a implémenter plus tard
			read_left_shell();
			read_right_shell();
			loop_rate.sleep();
		}
	}


};


#endif
