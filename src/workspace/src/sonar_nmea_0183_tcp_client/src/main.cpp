#ifndef MAIN_CPP
#define MAIN_CPP

#include <iostream>
#include <exception>

//sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//ROS
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"


class Sonar{

	private:
		ros::NodeHandle node;
		ros::Publisher sonarTopic;

		uint32_t sequenceNumber;

		std::string serverAddress;
		std::string serverPort;

		bool useDepth = true;
		bool usePosition = false;
		bool useAttitude = false;


	public:
		Sonar(char * serverAddress, char * serverPort, bool useDepth,bool usePosition,bool useAttitude) : serverAddress(serverAddress),serverPort(serverPort),useDepth(useDepth),usePosition(usePosition),useAttitude(useAttitude){

		}

		void run(){

			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);

			ros::Rate retry_rate(1);

		        while(ros::ok()){
				int s = -1;

				try{
					std::cout << "Connecting to " << serverAddress << ":" << serverPort << "..." << std::endl;

					struct addrinfo sa,*res;

					memset(&sa,0,sizeof(sa));

					getaddrinfo(serverAddress.c_str(),serverPort.c_str(),&sa,&res);

                                        if((s = socket(res->ai_family,res->ai_socktype,res->ai_protocol))==-1){
                                                perror("socket");
                                                throw std::runtime_error("socket");
                                        }


					if(connect(s,res->ai_addr,res->ai_addrlen) == -1){
						perror("connect");
						throw std::runtime_error("connect");
					}


					//read char by char like a dumbass
					char ch;
					std::string line;

					//FIXME: Holy wasted-syscalls Batman, that's inefficient!
					while(read(s,&ch,1)==1){

						if(ros::isShuttingDown()){
							close(s);
						}

						if(ch == '\n'){
							char talkerId[2];
							double  depthFeet;
							double  depthMeters;
							double  depthFathoms;
							unsigned int checksum;

							//Lookout for DBT strings such as
							// $SDDBT,30.9,f,9.4,M,5.1,F*35
							if(sscanf(line.c_str(),"$%2sDBT,%lf,f,%lf,M,%lf,F*%2x",talkerId,&depthFeet,&depthMeters,&depthFathoms,&checksum) == 5){

								//TODO: checksum
								//process depth

								geometry_msgs::PointStamped msg;

                        			                msg.header.seq=++sequenceNumber;
								msg.header.stamp=ros::Time::now();

								msg.point.z = depthMeters;

								sonarTopic.publish(msg);

								//std::cout << line << std::endl;
								//std::cout << depthMeters << std::endl;
							}

							//TODO: parse GGA and attitude strings

							line = "";
						}
						else{
							line.append(1,ch);
						}
					}

					close(s);
					s = -1;
				}
				catch(std::exception& e){
					if(s != -1) close(s);

					s = -1;
					retry_rate.sleep();
				}
        		}
		}

};


int main(int argc,char** argv){
	ros::init(argc, argv, "sonar");

	std::string addr;
	int port;

	//if no params present. use default values of 127.0.0.1:5000
	if(!ros::param::get("ip_address", addr)){
		addr = "127.0.0.1";
	}

	if(!ros::param::get("port", port)){
		port = 5000;
	}

	int a,b,c,d;

	//verify IP address
	if(
		sscanf(addr.c_str(),"%d.%d.%d.%d",&a,&b,&c,&d)==4 &&
		a > 0 && a <= 255 &&
		b >= 0 && b <= 255 &&
		c >= 0 && c <= 255 &&
		d >= 0 && d <= 255 
	){

		if(
			port > 0 && port <= 65535
		){
			//TODO: get useDepth/usePOsition/useAttitude from parameters
			Sonar sonar(argv[1],argv[2],true,true,true);
			sonar.run();
		}
		else{
			std::cerr << "Bad TCP port: " << port << std::endl;
		}
	}
	else{
		std::cerr << "Bad IP address: " << addr << std::endl;
	}
}

#endif
