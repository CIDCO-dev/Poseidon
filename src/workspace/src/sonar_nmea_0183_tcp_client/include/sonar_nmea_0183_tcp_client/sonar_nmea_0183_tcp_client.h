#ifndef sonar_nmea_0183_tcp_client
#define sonar_nmea_0183_tcp_client


#include <iostream>
#include <exception>

//sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//POSIX files
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


//ROS
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"

#include "../../utils/string_utils.hpp"



typedef struct{
	char talkerId[2];
	double utcTime;
	
	double latitude; 
	char northOrSouth;
	
	double longitude; 
	char eastOrWest;
	
	int quality;
	
	int nbSatellites;
	
	double hdop;
	
	double antennaAltitude;
	
	double geoidalSeparation;
	
	int dgpsAge;
	int dgpsStationId;	
	
	unsigned int checksum;
} ggaData;

typedef struct{
	char talkerId[2];
	double  depthFeet;
	double  depthMeters;
	double  depthFathoms;
	unsigned int checksum;
}dbtData;

typedef struct{
	char talkerId[2];
	double degreesDecimal;
	double degreesMagnetic;
	double speedKnots;
	double speedKmh;
	unsigned int checksum;
}vtgData;

typedef struct{
	char talkerId[2];
	double  depthMeters;
	double  offsetMeters; 
	double  maxRangeScale; // i guess it's meters
	unsigned int checksum;
}dptData;


class BaseNmeaClient{
	
	protected:
		ros::NodeHandle node;
	
	private:
		ros::Publisher sonarTopic;
		ros::Publisher gnssTopic;
		ros::Publisher speedTopic;

		uint32_t depthSequenceNumber = 0;
		uint32_t gpsSequenceNumber = 0;
		uint32_t speedSequenceNumber = 0;
			
		bool useDepth = true;
		bool usePosition = false;
		bool useAttitude = false;


	public:
		BaseNmeaClient(bool useDepth,bool usePosition,bool useAttitude) : useDepth(useDepth),usePosition(usePosition),useAttitude(useAttitude){

		}
		
		static uint8_t computeChecksum(std::string &data){
			
			uint8_t checksum = 0;
			for(int i=0; i<data.size(); i++){
				checksum ^= data.at(i);
			}
			return checksum;
		}
		
		static uint8_t str2checksum(std::string checksumByte){
			uint8_t msb = checksumByte.at(0);
			uint8_t lsb = checksumByte.at(1);

			if(msb >= '0' && msb <= '9') msb = msb - '0';
			else if(msb >='A' && msb <='F') msb = msb -'A'+10;
			else return false;
			msb = msb << 4;
			
			if(lsb >= '0' && lsb <= '9') lsb = lsb - '0';
			else if(lsb >='A' && lsb <='F') lsb = lsb -'A'+10;
			else return false;
			
			return msb + lsb;
		}
		static bool validateChecksum(std::string &s){
			std::string bytes = s.substr(s.find("$")+1, s.find("*")-1);
			uint8_t checksum = computeChecksum(bytes);
			
			std::string checksumByte = s.substr(s.find("*")+1, s.find("*")+2);
			uint8_t controlByte = str2checksum(checksumByte);
			
			if(checksum == controlByte){
				return true;
			}
			else{
				return false;
			}
		}
		
		void initTopics(){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
			gnssTopic  = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
			speedTopic = node.advertise<nav_msgs::Odometry>("speed",1000);		
		}
		//$GPGGA,133818.75,0100.0000,N,00300.0180,E,1,14,3.1,-13.0,M,-45.3,M,,*52'
		bool extractGGA(std::string & s){   
			ggaData data;
			
			if(sscanf(s.c_str(),"$%2sGGA,%lf,%lf,%1s,%lf,%1s,%d,%d,%lf,%lf,M,%lf,M,%d,%d*%2x",&data.talkerId,&data.utcTime,&data.latitude,&data.northOrSouth,&data.longitude,&data.eastOrWest,&data.quality,&data.nbSatellites,&data.hdop,&data.antennaAltitude,&data.geoidalSeparation,&data.dgpsAge,&data.dgpsStationId,&data.checksum) >= 8){		
				//TODO verify checksum
				if(validateChecksum(s)){
					sensor_msgs::NavSatFix msg;
					msg.header.seq=++gpsSequenceNumber;
					msg.header.stamp=ros::Time::now();			
					int longDegrees =( (double)data.longitude / 100);
					double longMinutes = (data.longitude - (longDegrees*100)) / (double)60;
					double sign = (data.eastOrWest=='E')?1:-1;
				
					msg.longitude = sign* (longDegrees + longMinutes );
				
					int latDegrees = (double)(data.latitude / 100);
					double latMinutes = (data.latitude - (latDegrees*100)) / (double)60;
					sign = (data.northOrSouth=='N')?1:-1;		

					msg.latitude  = sign * (latDegrees+latMinutes);
															
					switch(data.quality){
						//No fix
						case 0:
							msg.status.status=-1;
							break;
																
						//GPS Fix
						case 1:
							msg.status.status=0;
							break;
																	
							//DGPS
						case 2:
							msg.status.status=2;
							break;
					}
															
					msg.altitude  = data.antennaAltitude;
					msg.position_covariance_type= 0;
																
					gnssTopic.publish(msg);

					return true;
					}
					else{
						ROS_ERROR("checksum error");
					}
				}
			
			return false;
		}		

		//parse DBT strings such as $SDDBT,30.9,f,9.4,M,5.1,F*35
		bool extractDBT(std::string & s){
			dbtData dbt;
			
			if(sscanf(s.c_str(),"$%2sDBT,%lf,f,%lf,M,%lf,F*%2x",&dbt.talkerId,&dbt.depthFeet,&dbt.depthMeters,&dbt.depthFathoms,&dbt.checksum) == 5){
				//TODO: checksum
				//process depth
				if(validateChecksum(s)){
					geometry_msgs::PointStamped msg;

					msg.header.seq=++depthSequenceNumber;
					msg.header.stamp=ros::Time::now();

					msg.point.z = dbt.depthMeters;

					sonarTopic.publish(msg);
				
				return true;
				}
				else{
					ROS_ERROR("checksum error");
				}
			}			
			
			return false;
		}
		
		bool extractVTG(std::string & s){
			vtgData vtg;
			//$GPVTG,82.0,T,77.7,M,2.4,N,4.4,K,S*3A\r\n
			if(sscanf(s.c_str(),"$%2sVTG,%lf,T,%lf,M,%lf,N,%lf,K,S*%2x",&vtg.talkerId,&vtg.degreesDecimal,&vtg.degreesMagnetic,&vtg.speedKnots,&vtg.speedKmh,&vtg.checksum) == 6 ){
				
				if(validateChecksum(s)){
					nav_msgs::Odometry msg;
					msg.header.seq=++speedSequenceNumber;
					msg.header.stamp=ros::Time::now();
					msg.twist.twist.linear.y=vtg.speedKmh;
					speedTopic.publish(msg);
					
					return true;
				}
				else{
					ROS_ERROR("checksum error");
				}
			}
			return false;
		}
		
		bool extractDPT(std::string & s){
			dptData dpt;

			//$INDPT,5.0,0.0,0.0*46\r\n
			if(sscanf(s.c_str(),"$%2sDPT,%lf,%lf,%lf,S*%2x",&dpt.talkerId,&dpt.depthMeters,&dpt.offsetMeters,&dpt.maxRangeScale,&dpt.checksum) == 4 ){
				if(validateChecksum(s)){
					geometry_msgs::PointStamped msg;
					msg.header.seq=++depthSequenceNumber;
					msg.header.stamp=ros::Time::now();
					msg.point.z = dpt.depthMeters;
					sonarTopic.publish(msg);
					return true;
				}
				else{
					ROS_ERROR("checksum error");
				}
			}
			
			else if(sscanf(s.c_str(),"$%2sDPT,%lf,%lf,S*%2x",&dpt.talkerId,&dpt.depthMeters,&dpt.offsetMeters,&dpt.checksum) == 3 ){
				if(validateChecksum(s)){
					geometry_msgs::PointStamped msg;
					msg.header.seq=++depthSequenceNumber;
					msg.header.stamp=ros::Time::now();
					msg.point.z = dpt.depthMeters;
					sonarTopic.publish(msg);
					return true;
				}
				else{
					ROS_ERROR("checksum error");
				}
			}
			return false;
		}
		
		

		void readStream(int & fileDescriptor){
			//read char by char like a dumbass
			char ch;
			std::string line;
			
			bool (BaseNmeaClient::* handlers[4]) (std::string &) {&BaseNmeaClient::extractDBT,&BaseNmeaClient::extractGGA,&BaseNmeaClient::extractVTG}; // More handlers can be added here

			//FIXME: Holy wasted-syscalls Batman, that's inefficient!
			while(read(fileDescriptor,&ch,1)==1){
				if(ros::isShuttingDown()){
					close(fileDescriptor);
				}

				if(ch == '\n'){
					//Run through the array of handlers, stopping either at the end or when one returns true
					//TODO: fix this for(long unsigned int i=0;i < (sizeof(handlers)/sizeof(handlers[0])) && !( (this->* handlers[i])(line) );i++);

					if(!extractDBT(line)){
						if(!extractGGA(line)){
							if(!extractVTG(line)){
								extractDPT(line);
							}
						}
					}

					line = "";
				}
				else{
					line.append(1,ch);
				}
			}

			close(fileDescriptor);
			fileDescriptor = -1;
		}

		virtual void run() = 0;
};

class NetworkNmeaClient : public BaseNmeaClient{
public:
	NetworkNmeaClient(char * serverAddress, char * serverPort, bool useDepth,bool usePosition,bool useAttitude) : BaseNmeaClient(useDepth,usePosition,useAttitude),serverAddress(serverAddress),serverPort(serverPort){
	
	}
	
	void run(){

		initTopics();

		ros::Rate retry_rate(1);

		while(ros::ok()){
			int s = -1;

			try{
				std::cout << "Connecting to NMEA network source " << serverAddress << ":" << serverPort << "..." << std::endl;

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


				readStream(s);
			}
			catch(std::exception& e){
				if(s != -1) close(s);

				s = -1;
				retry_rate.sleep();
			}
		}
	}
		
private:
	std::string serverAddress;
	std::string serverPort;
};

class DeviceNmeaClient : public BaseNmeaClient{
public:
	DeviceNmeaClient(std::string & deviceFile, bool useDepth,bool usePosition,bool useAttitude) : BaseNmeaClient(useDepth,usePosition,useAttitude),deviceFile(deviceFile){
	
	configurationSubscriber = node.subscribe("configuration", 1000, &DeviceNmeaClient::configurationCallBack, this);
	configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
	
	}
	
	void init_serial_port_speed(){
		setting_msg::ConfigurationService srv;

		srv.request.key = "sonarSerialBaudRate";

		if(configurationClient.call(srv)){
			std::string baudRate = srv.response.value;
			try{
				if(std::stoi(baudRate) != this->sonarSerialBaudRate){
					this->sonarSerialBaudRate = std::stoi(baudRate);
				}
			}
			catch(const std::exception& ex){
				ROS_ERROR_STREAM(ex.what());
				ROS_ERROR("Error in sonar serial baud rate definition, defaulting to 9600");
				this->sonarSerialBaudRate = 9600;
			}
		}
		else{
			ROS_WARN("No sonar serial baud rate definition, defaulting to 9600");
			this->sonarSerialBaudRate = 9600;
		}
		
		set_sonar_baud_rate();
	}
	
	void run(){
		initTopics();

		try{
			std::cout << "Opening NMEA device " << deviceFile << std::endl;

			if((this->fileDescriptor = open(deviceFile.c_str(),O_RDWR))==-1){
				perror("open");
				throw std::runtime_error("open");
			}
			
			init_serial_port_speed();
			speed_t outBaudRate, inBaudRate;
			if(getBaudRateSetting(&outBaudRate, &inBaudRate)){
				ROS_INFO_STREAM("Current baud rate for sonar is, input speed: " << inBaudRate << " out: " << outBaudRate);
			}
			
			readStream(this->fileDescriptor);
		}
		catch(std::exception& e){
			if(this->fileDescriptor != -1) close(this->fileDescriptor);

			std::cerr << e.what() << std::endl;
		}
	}
	
	void configurationCallBack(const setting_msg::Setting &setting){
		if(setting.key == "sonarSerialBaudRate"){
			std::string value = setting.value;
			if(trimSpaces(value) == ""){
				this->sonarSerialBaudRate = 9600;
				ROS_INFO_STREAM("No baud rate specify in config file for sonar \n Defaulting to 9600");
			}
			else{
				try{
					if(stoi(setting.value) != this->sonarSerialBaudRate){
						std::string value = setting.value;
						this->sonarSerialBaudRate = stoi(trimSpaces(value));
						set_sonar_baud_rate();
					}
					//else value is same, do nothing
				}
				catch(std::invalid_argument &err){
					ROS_ERROR_STREAM("Baud rate from configuration is not set properly \n example : 9600");
					speed_t outBaudRate, inBaudRate;
					if(getBaudRateSetting(&outBaudRate, &inBaudRate)){
						ROS_INFO_STREAM("Current baud rate for sonar is, input speed: " << inBaudRate << " out: " << outBaudRate);
					}
				}
			}
		}
	}
	
	void set_sonar_baud_rate(){
		
		struct termios tty;
		if (tcgetattr(this->fileDescriptor, &tty) != -1) {
			ROS_ERROR_STREAM("Error getting sonar serial port attributes");
			return;
		}

		cfsetospeed(&tty, getBaudRate(this->sonarSerialBaudRate));
		cfsetispeed(&tty, getBaudRate(this->sonarSerialBaudRate));

		if (tcsetattr(this->fileDescriptor, TCSANOW, &tty) != -1) {
			ROS_ERROR_STREAM("Error setting sonar serial port attributes");
			return;
		}
		
		
		ROS_INFO_STREAM("Sonar baud rate set to : " << this->sonarSerialBaudRate);
	}
	
	speed_t getBaudRate(int baudrate) {
		switch(baudrate) {
			case 50:	return B50;
			case 75:	return B75;
			case 110:   return B110;
			case 134:   return B134;
			case 150:   return B150;
			case 200:   return B200;
			case 300:   return B300;
			case 600:   return B600;
			case 1200:  return B1200;
			case 1800:  return B1800;
			case 2400:  return B2400;
			case 4800:  return B4800;
			case 9600:  return B9600;
			case 19200: return B19200;
			case 38400: return B38400;
			case 57600: return B57600;
			case 115200:return B115200;
			default:	return -1; // Invalid baud rate
		}
	}
	
	bool getBaudRateSetting(speed_t *outBaudRate, speed_t *inBaudRate) {
		struct termios tty;
		if (tcgetattr(this->fileDescriptor, &tty) != -1 ) {
			ROS_ERROR_STREAM("Error getting sonar serial port attributes");
			return false;
		}

		*outBaudRate = cfgetospeed(&tty);
		*inBaudRate = cfgetispeed(&tty);

		return true;
	}
	
private:
	int fileDescriptor = -1;
	std::string deviceFile;
	int sonarSerialBaudRate = 9600;
	ros::Subscriber configurationSubscriber;
	ros::ServiceClient configurationClient;
};

#endif
