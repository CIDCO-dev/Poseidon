#include "catarob_control/catarob_control.h"

	void CATAROB::i2ctransmit(char id,char pwm,char imaxl,char imaxh,char relay){
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
    		unsigned char checksum=99;
    		for(int i=1;i<=13;i++){
    			checksum = checksum xor WRbuffer[i];
    			}
    		WRbuffer[14] = checksum;

    		//écriture des valeur sur le port i2c
    		mtx.lock();
    		wiringPiI2CWriteReg8 (id, 0x00, WRbuffer[1]);
    		wiringPiI2CWriteReg8 (id, 0x01, WRbuffer[2]);
    		wiringPiI2CWriteReg8 (id, 0x02, WRbuffer[3]);
    		wiringPiI2CWriteReg8 (id, 0x03, WRbuffer[4]);
    		wiringPiI2CWriteReg8 (id, 0x04, WRbuffer[5]);
    		wiringPiI2CWriteReg8 (id, 0x05, WRbuffer[6]);
    		wiringPiI2CWriteReg8 (id, 0x06, WRbuffer[7]);
    		wiringPiI2CWriteReg8 (id, 0x07, WRbuffer[8]);
    		wiringPiI2CWriteReg8 (id, 0x08, WRbuffer[9]);
    		wiringPiI2CWriteReg8 (id, 0x09, WRbuffer[10]);
    		wiringPiI2CWriteReg8 (id, 0x0A, WRbuffer[11]);
    		wiringPiI2CWriteReg8 (id, 0x0B, WRbuffer[12]);
    		wiringPiI2CWriteReg8 (id, 0x0C, WRbuffer[13]);
    		wiringPiI2CWriteReg8 (id, 0x0D, WRbuffer[14]);
    		mtx.unlock();
		free(WRbuffer);
		}

	void CATAROB::i2crecive(char id){
		catarob_control::state msg1;
    		mtx.lock();
		msg1.status	= (unsigned short)wiringPiI2CReadReg8(id, 0x02)*256+wiringPiI2CReadReg8(id, 0x03);
		ADCTempVal 	= wiringPiI2CReadReg8(id, 0x04)*256+wiringPiI2CReadReg8(id, 0x05);
		if (ADCTempVal<=1023){
       			double Ve_double=0;
       			uint16_t x1,x2,index;
       			int y1;
       			Ve_double=ADCTempVal;
       			for(index=0;index<30;index++){
            			if( ADCTempVal<=pt_cei012c_tab_ech[index])  //si la valeur du tab de ref est <= on sort de la boucle
            			break;
            			}
        		if(index==0) ADCTempVal= 0; //si la la case est la premiere on est < a -20ï¿½C on renvoit -40ï¿½C par defaut
           		else{
          			y1 = (index-1)*5 - 20;
          			x1 = pt_cei012c_tab_ech[index-1];
          			x2 = pt_cei012c_tab_ech[index];
          			Ve_double = y1+(5.0*((double)(ADCTempVal-x1))/((double)(x2-x1)));
          			msg1.temp=Ve_double;
          			}
      			}
		msg1.pwm	= wiringPiI2CReadReg8(id, 0x06)*256+wiringPiI2CReadReg8(id, 0x07);
		msg1.pmw_rc	= wiringPiI2CReadReg8(id, 0x08)*256+wiringPiI2CReadReg8(id, 0x09);
		msg1.pwmsource	= wiringPiI2CReadReg8(id, 0x0A)*256+wiringPiI2CReadReg8(id, 0x0B);
		msg1.ibat	= 0.0732601*(double)(wiringPiI2CReadReg8(id, 0x0C)*256+wiringPiI2CReadReg8(id, 0x0D)) - 37.4847189;
		msg1.imot	= 0.0732601*(double)(wiringPiI2CReadReg8(id, 0x0E)*256+wiringPiI2CReadReg8(id, 0x0F)) - 37.4847189;
		msg1.vbat	= 0.0167343*(double)(wiringPiI2CReadReg8(id, 0x10)*256+wiringPiI2CReadReg8(id, 0x11));
		msg1.imax	= 0.0732601*(double)(wiringPiI2CReadReg8(id, 0x12)*256+wiringPiI2CReadReg8(id, 0x13)) - 37.4847189;
		msg1.tor_rc	= wiringPiI2CReadReg8(id, 0x14)*256+wiringPiI2CReadReg8(id, 0x15);
		msg1.water	= wiringPiI2CReadReg8(id, 0x16)*256+wiringPiI2CReadReg8(id, 0x17);
		msg1.relay	= wiringPiI2CReadReg8(id, 0x18)*256+wiringPiI2CReadReg8(id, 0x19);
		msg1.v5v	= (double)(wiringPiI2CReadReg8(id, 0x1A)*256+wiringPiI2CReadReg8(id, 0x1B))/100.0;
		msg1.v12v	= (double)(wiringPiI2CReadReg8(id, 0x1C)*256+wiringPiI2CReadReg8(id, 0x1D))/100.0;
		msg1.batv	= (double)(wiringPiI2CReadReg8(id, 0x1E)*256+wiringPiI2CReadReg8(id, 0x1F))/100.0;
		msg1.v24v	= (double)(wiringPiI2CReadReg8(id, 0x20)*256+wiringPiI2CReadReg8(id, 0x21))/100.0;
		msg1.pwm_modbus	= wiringPiI2CReadReg8(id, 0x22)*256+wiringPiI2CReadReg8(id, 0x23);
		msg1.relay_cmd		= wiringPiI2CReadReg8(id, 0x24)*256+wiringPiI2CReadReg8(id, 0x25);
		msg1.mot_limit 	= wiringPiI2CReadReg8(id, 0X26)*256+wiringPiI2CReadReg8(id, 0x27);
    		mtx.unlock();
		if (id == fdL) state_L.publish(msg1);
		if (id == fdR) state_R.publish(msg1);
		}

	void CATAROB::motorLcallback(const catarob_control::motor& motor_L)
		{
		i2ctransmit(fdL,motor_L.pwm,motor_L.imaxl,motor_L.imaxh,motor_L.relay);
		}

	void CATAROB::motorRcallback(const catarob_control::motor& motor_R)
		{
		i2ctransmit(fdR,motor_R.pwm,motor_R.imaxl,motor_R.imaxh,motor_R.relay);
		}
