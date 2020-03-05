#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime> 
#include <string>
#include <serial/serial.h>
#include <std_msgs/String.h>

using namespace std;

string fnameout="123";
string path="123";


class ZEDF9P{
	private:
	std::string outputFolder;
	std::string serialport;
	serial::Serial ser;
	std_msgs::String result;

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
	

	void run(){
		ser.setPort(serialport);
        	ser.setBaudrate(115200);
		ser.open();
		//écrit la configuration du module gps


		while(ros::ok()){
			//lis le temps si la seconde change on change le nom du fichier
			fnameout = outputFolder + datetime() +".ubx";
		
			//resoit les donnée du port série /dev/ttyACM0
			result.data = ser.read(ser.available());

			//ouvre le fichier
			ofstream file;
			file.open(fnameout,ios::app); // File output stream, the file that we'll write to it
			if(!file) { // if can't open file
			}
			//écrit les donée recu du port série dans le fichier
			file << result.data;
			//ferme le fichier
			file.close();
		
		}
		ser.close();
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
