#ifndef catarob_control
#define catarob_control

#include "ros/ros.h"
#include "catarob_control/motor.h"
#include "catarob_control/state.h"


#include <wiringPiI2C.h>

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;

int fdL,fdR;
unsigned short pt_cei012c_tab_ech[30] = {96,124,157,196,241,290,343,398,455,512,567,619,668,712,753,788,820,847,871,892,910,925,938,949,959,967,974,981,986,990};
uint16_t ADCTempVal = 0;
std::mutex mtx;

class CATAROB{
	private:
		
	public:
		CATAROB(void){
			ros::Subscriber motor_L = n.subscribe("motor/left", 1000, &CATAROB::motorLcallback,this);
			ros::Subscriber motor_R = n.subscribe("motor/right", 1000, &CATAROB::motorRcallback,this);
			state_L = n.advertise<catarob_control::state>("state/left", 1000);
			state_R = n.advertise<catarob_control::state>("state/right", 1000);
			
			
			
		}
		void i2ctransmit(char id,char pwm,char imaxl,char imaxh,char relay);
		void i2crecive(char id);
		void motorLcallback(const catarob_control::motor& motor_L);
        	void motorRcallback(const catarob_control::motor& motor_R);
		
		
};


#endif
