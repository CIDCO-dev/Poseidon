#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"

#include <iostream>



class MOTOR{
	private:
	
	public:

	MOTOR(){
		}



	void run(){
		while(ros::ok()){
		//lis le temps si la seconde change on change le nom du fichier
		//resoit les donn�e du port s�rie /dev/ttyACM0
		//ouvre le fichier
		//�crit les don�e recu du port s�rie dans le fichier	
		//ferme le fichier	
		}
	}


};
int main(int argc,char** argv){
	ros::init(argc, argv, "motor");
	//initialiser le module zed-f9p

	MOTOR motor;
	motor.run();
	ros::spin();
}
#endif
