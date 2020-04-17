#include "ros/ros.h"
#include "catarob_control/catarob_control.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "motor");
	fdL = wiringPiI2CSetup(0x18); //Adresse controleur gauche
	fdR = wiringPiI2CSetup(0x10); //Asresse controleur droit
	CATAROB catarob;
	motor.run();
	ros::spin();
	ros::Rate loop_rate( 1 );
	while(ros::ok()){
		i2crecive(fdL);
		i2crecive(fdR);
		loop_rate.sleep();
	}
	return 0;
}

