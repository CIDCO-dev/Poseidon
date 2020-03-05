#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime> 
#include <string>

using namespace std;

string fnameout="123";
string path="123";

class MOTOR{
	private:
	
	public:

	MOTOR(){
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


	void run(){
		while(ros::ok()){
		//lis le temps si la seconde change on change le nom du fichier
		path = "/home/ubuntu/ubx/";

		fnameout = path + datetime() +".ubx";
		//resoit les donn�e du port s�rie /dev/ttyACM0
		

		//ouvre le fichier
		ofstream file;
		file.open(fnameout); // File output stream, the file that we'll write to it
		if(!file) { // if can't open file
		}
		//�crit les don�e recu du port s�rie dans le fichier
		file << 1 << endl;
		//ferme le fichier
		file.close();
	
		}
	}


};
int main(int argc,char** argv){
  	if(argc < 2){
    		std::cout << "nlogger_text, Missing output folder path" << std::endl;
    		return 1;
  	}
	std::string outputFolder( argv[1] );
	
	outputFolder = "/home/ubuntu/ubx/";

	ros::init(argc, argv, "motor");

	//r�cup�ration du path
	//cr�ation du path du log si non pr�sent
	;
	//initialiser le module zed-f9p

	MOTOR motor;
	motor.run();
	ros::spin();
}
#endif
