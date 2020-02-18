#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "catarob_control/motorL.h"
#include "catarob_control/motorR.h"
#include "catarob_control/stateL.h"
#include "catarob_control/stateR.h"

#include <wiringPiI2C.h>
  
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;

int fdL,fdR;
unsigned short pt_cei012c_tab_ech[30] = {96,124,157,196,241,290,343,398,455,512,567,619,668,712,753,788,820,847,871,892,910,925,938,949,959,967,974,981,986,990};
uint16_t ADCTempVal = 0;

void motorLcallback(const catarob_control::motorL)
{
//   fd = wiringPiI2CSetup(0x60);
//result = wiringPiI2CWriteReg16(fd, 0x40, (i & 0xfff) );//
  }

void motorRcallback(const catarob_control::motorR)
{
  }

class MOTOR{
	private:
		ros::NodeHandle n;
		ros::Publisher state_L;
		ros::Publisher state_R;
		
		
	public:
		MOTOR(){
		ros::Subscriber motor_L = n.subscribe("motor/left", 1000, motorLcallback);
		ros::Subscriber motor_R = n.subscribe("motor/right", 1000, motorRcallback);
		state_L= n.advertise<catarob_control::stateL>("state/left", 1000);
		state_R= n.advertise<catarob_control::stateR>("state/right", 1000);
		}
		
	void run(){
		ros::Rate loop_rate(1);
		while(ros::ok()){
			

			
			catarob_control::stateL msg1;
			msg1.status	= (unsigned short)wiringPiI2CReadReg8(fdL, 0x02)*256+wiringPiI2CReadReg8(fdL, 0x03);
			ADCTempVal 	= wiringPiI2CReadReg8(fdL, 0x04)*256+wiringPiI2CReadReg8(fdL, 0x05);
			if (ADCTempVal<=1023){
           			double Ve_double=0;
            			uint16_t x1,x2,index;
            			int y1;
            			Ve_double=ADCTempVal;
            			for(index=0;index<30;index++){
                    			if( ADCTempVal<=pt_cei012c_tab_ech[index])  //si la valeur du tab de ref est <= on sort de la boucle
                    			break;
            			}

            			if(index==0)
                    			ADCTempVal= 0; //si la la case est la premiere on est < a -20°C on renvoit -40°C par defaut
            			else{
                    			y1 = (index-1)*5 - 20;
                    			x1 = pt_cei012c_tab_ech[index-1];
                    			x2 = pt_cei012c_tab_ech[index];
                    			Ve_double = y1+(5.0*((double)(ADCTempVal-x1))/((double)(x2-x1)));
                    			msg1.temp=Ve_double;
            			}
        		}
			msg1.pwm	= wiringPiI2CReadReg8(fdL, 0x06)*256+wiringPiI2CReadReg8(fdL, 0x07);
			msg1.pmw_rc	= wiringPiI2CReadReg8(fdL, 0x08)*256+wiringPiI2CReadReg8(fdL, 0x09);
			msg1.pwmsource	= wiringPiI2CReadReg8(fdL, 0x0A)*256+wiringPiI2CReadReg8(fdL, 0x0B);
			msg1.ibat	= 0.0732601*(double)(wiringPiI2CReadReg8(fdL, 0x0C)*256+wiringPiI2CReadReg8(fdL, 0x0D)) - 37.4847189;
			msg1.imot	= 0.0732601*(double)(wiringPiI2CReadReg8(fdL, 0x0E)*256+wiringPiI2CReadReg8(fdL, 0x0F)) - 37.4847189;
			msg1.vbat	= 0.0167343*(double)(wiringPiI2CReadReg8(fdL, 0x10)*256+wiringPiI2CReadReg8(fdL, 0x11));
			msg1.imax	= 0.0732601*(double)(wiringPiI2CReadReg8(fdL, 0x12)*256+wiringPiI2CReadReg8(fdL, 0x13)) - 37.4847189;
			msg1.tor_rc	= wiringPiI2CReadReg8(fdL, 0x14)*256+wiringPiI2CReadReg8(fdL, 0x15);
			msg1.water	= wiringPiI2CReadReg8(fdL, 0x16)*256+wiringPiI2CReadReg8(fdL, 0x17);
			msg1.relay	= wiringPiI2CReadReg8(fdL, 0x18)*256+wiringPiI2CReadReg8(fdL, 0x19);
			msg1.v5v	= (double)(wiringPiI2CReadReg8(fdL, 0x1A)*256+wiringPiI2CReadReg8(fdL, 0x1B))/100.0;
			msg1.v12v	= (double)(wiringPiI2CReadReg8(fdL, 0x1C)*256+wiringPiI2CReadReg8(fdL, 0x1D))/100.0;
			msg1.batv	= (double)(wiringPiI2CReadReg8(fdL, 0x1E)*256+wiringPiI2CReadReg8(fdL, 0x1F))/100.0;
			msg1.v24v	= (double)(wiringPiI2CReadReg8(fdL, 0x20)*256+wiringPiI2CReadReg8(fdL, 0x21))/100.0;
			msg1.pwm_modbus	= wiringPiI2CReadReg8(fdL, 0x22)*256+wiringPiI2CReadReg8(fdL, 0x23);
			msg1.relay_cmd	= wiringPiI2CReadReg8(fdL, 0x24)*256+wiringPiI2CReadReg8(fdL, 0x25);
			msg1.mot_limit 	= wiringPiI2CReadReg8(fdL, 0X26)*256+wiringPiI2CReadReg8(fdL, 0x27);
			state_L.publish(msg1);
			
			
			catarob_control::stateR msg2;
			msg2.status	= (unsigned short)wiringPiI2CReadReg8(fdR, 0x02)*256+wiringPiI2CReadReg8(fdR, 0x03);
			ADCTempVal 	= wiringPiI2CReadReg8(fdR, 0x04)*256+wiringPiI2CReadReg8(fdR, 0x05);
			if (ADCTempVal<=1023){
           			double Ve_double=0;
            			uint16_t x1,x2,index;
            			int y1;
            			Ve_double=ADCTempVal;
            			for(index=0;index<30;index++){
                    			if( ADCTempVal<=pt_cei012c_tab_ech[index])  //si la valeur du tab de ref est <= on sort de la boucle
                    			break;
            			}

            			if(index==0)
                    			ADCTempVal= 0; //si la la case est la premiere on est < a -20°C on renvoit -40°C par defaut
            			else{
                    			y1 = (index-1)*5 - 20;
                    			x1 = pt_cei012c_tab_ech[index-1];
                    			x2 = pt_cei012c_tab_ech[index];
                    			Ve_double = y1+(5.0*((double)(ADCTempVal-x1))/((double)(x2-x1)));
                    			msg2.temp=Ve_double;
            			}
        		}
			msg2.pwm	= wiringPiI2CReadReg8(fdR, 0x06)*256+wiringPiI2CReadReg8(fdR, 0x07);
			msg2.pmw_rc	= wiringPiI2CReadReg8(fdR, 0x08)*256+wiringPiI2CReadReg8(fdR, 0x09);
			msg2.pwmsource	= wiringPiI2CReadReg8(fdR, 0x0A)*256+wiringPiI2CReadReg8(fdR, 0x0B);
			msg2.ibat	= 0.0732601*(double)(wiringPiI2CReadReg8(fdR, 0x0C)*256+wiringPiI2CReadReg8(fdR, 0x0D)) - 37.4847189;
			msg2.imot	= 0.0732601*(double)(wiringPiI2CReadReg8(fdR, 0x0E)*256+wiringPiI2CReadReg8(fdR, 0x0F)) - 37.4847189;
			msg2.vbat	= 0.0167343*(double)(wiringPiI2CReadReg8(fdR, 0x10)*256+wiringPiI2CReadReg8(fdR, 0x11));
			msg2.imax	= 0.0732601*(double)(wiringPiI2CReadReg8(fdR, 0x12)*256+wiringPiI2CReadReg8(fdR, 0x13)) - 37.4847189;
			msg2.tor_rc	= wiringPiI2CReadReg8(fdR, 0x14)*256+wiringPiI2CReadReg8(fdR, 0x15);
			msg2.water	= wiringPiI2CReadReg8(fdR, 0x16)*256+wiringPiI2CReadReg8(fdR, 0x17);
			msg2.relay	= wiringPiI2CReadReg8(fdR, 0x18)*256+wiringPiI2CReadReg8(fdR, 0x19);
			msg2.v5v	= (double)(wiringPiI2CReadReg8(fdR, 0x1A)*256+wiringPiI2CReadReg8(fdR, 0x1B))/100.0;
			msg2.v12v	= (double)(wiringPiI2CReadReg8(fdR, 0x1C)*256+wiringPiI2CReadReg8(fdR, 0x1D))/100.0;
			msg2.batv	= (double)(wiringPiI2CReadReg8(fdR, 0x1E)*256+wiringPiI2CReadReg8(fdR, 0x1F))/100.0;
			msg2.v24v	= (double)(wiringPiI2CReadReg8(fdR, 0x20)*256+wiringPiI2CReadReg8(fdR, 0x21))/100.0;
			msg2.pwm_modbus	= wiringPiI2CReadReg8(fdR, 0x22)*256+wiringPiI2CReadReg8(fdR, 0x23);
			msg2.relay_cmd	= wiringPiI2CReadReg8(fdR, 0x24)*256+wiringPiI2CReadReg8(fdR, 0x25);
			msg2.mot_limit 	= wiringPiI2CReadReg8(fdR, 0X26)*256+wiringPiI2CReadReg8(fdR, 0x27);
			state_R.publish(msg2);

			loop_rate.sleep();


		}

	}


};
int main(int argc,char** argv){
	ros::init(argc, argv, "motor");
	
	fdL = wiringPiI2CSetup(0x18);
	fdR = wiringPiI2CSetup(0x10);
	MOTOR motor;
	motor.run();


 	
	

	ros::spin();


}
#endif
