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

	public:
		Sonar(char * serverAddress, char * serverPort) : serverAddress(serverAddress),serverPort(serverPort){

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
#endif
int s;
char buf[INET_ADDRSTRLEN];
int main(int argc,char** argv){
	ros::init(argc, argv, "sonar");
	
	//TODO: Get params from command line
        s = inet_pton(AF_INET, argv[1], buf);
           if (s <= 0) {
               if (s == 0)
                   fprintf(stderr, "Bad Ip format : xxx.xxx.xxx.xxx");
               else
                   perror("inet_pton");
               exit(EXIT_FAILURE);
           }
	
	if (*argv[2]<1024 && *argv[2]>65535) {
		fprintf(stderr, "Port mus be 1024 to 65535");
		exit(EXIT_FAILURE);  
	   }
		
	Sonar sonar(argv[1],argv[2]);
	sonar.run();


}

