#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include <unistd.h>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


//TODO: init serial port with: sudo stty -F /dev/ttyUSB0 cs8 -parenb -cstopb clocal -crtscts -echo speed 115200

#pragma pack(1)
typedef struct{
	uint8_t magic[2];
	uint8_t headId;
	uint8_t range;
	uint8_t reserved[2];
	uint8_t masterSlave;
	uint8_t reserved2;
	uint8_t startGain;
	uint8_t reserved3;
	uint8_t absorption; //20 = 0.2db    675kHz
	uint8_t reserved4[3];
	uint8_t pulseLength; //1-255 -> 1us to 255us in 1us increments
	uint8_t profileMinimumRange; //Min range in meters / 10
	uint8_t reserved5[2];
	uint8_t triggerControl;
	uint8_t dataPoints;
	uint8_t reserved6[2];
	uint8_t profile;
	uint8_t reserved7;
	uint8_t switchDelay;
	uint8_t frequency;
	uint8_t terminationByte;

} Imagenex852SwitchDataCommand;

#pragma pack()


#pragma pack(1)
typedef struct{
	uint8_t magic[3];
	uint8_t headId;
	uint8_t serialStatus;
	uint8_t reserved[2];
	uint8_t range;
	uint8_t profileRange;
	uint8_t dataBytes[2];
} Imagenex852ReturnDataHeader;

#pragma pack()

class Imagenex852{
	public:
		Imagenex852(std::string devicePath) : devicePath(devicePath){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
		}

		~Imagenex852(){
			if(deviceFile>=0) close(deviceFile);
		}

		void run(){

			deviceFile = open(devicePath.c_str(),O_RDWR);

			if(deviceFile >= 0){
				ROS_INFO("Sonar file opened on %s",devicePath.c_str());

				ros::Rate loop_rate( 1 );

			        while(ros::ok()){
        	        		geometry_msgs::PointStamped msg;

			                msg.header.seq=sequenceNumber++;
                			msg.header.stamp=ros::Time::now();
					msg.header.frame_id = "sonar";

        		        	msg.point.z = measureDepth(500);

                			sonarTopic.publish(msg);

					ros::spinOnce();
			                loop_rate.sleep();
        			}
			}
			else{
				ROS_ERROR("Error while opening %s : %s",devicePath.c_str(),strerror(errno));
				throw std::invalid_argument(strerror(errno));
			}
		}

		double measureDepth(int dataPoints){
			Imagenex852SwitchDataCommand cmd;
			memset(&cmd,0,sizeof(Imagenex852SwitchDataCommand));

		        cmd.magic[0]    = 0xFE    ;
			cmd.magic[1]    = 0x44    ;
			cmd.headId      = 0x11    ;
			cmd.range       = 32      ;
			cmd.masterSlave = 0x43    ;
			cmd.startGain   = 0x06    ; 
			cmd.absorption  = 0x14    ; //20 = 0.2db    675kHz
			cmd.pulseLength = 150     ; //1-255 -> 1us to 255us in 1us increments
			cmd.profileMinimumRange =  0; //Min range in meters / 10
			cmd.triggerControl = 0x02;
			cmd.dataPoints  = dataPoints/10;
			cmd.profile     = 0;
			cmd.switchDelay = 0;
			cmd.frequency   = 0;
			cmd.terminationByte = 0xFD;

			unsigned int nbBytes;

			if( (nbBytes = write(deviceFile,&cmd,sizeof(Imagenex852SwitchDataCommand))) == 27){

				ROS_INFO("Sent command (%d bytes)",nbBytes);

				Imagenex852ReturnDataHeader hdr;

				if( (nbBytes = read(deviceFile,&hdr,sizeof(Imagenex852ReturnDataHeader))) == 11){

					ROS_INFO("Read %d bytes header",nbBytes);

					ROS_INFO("%c%c%c",hdr.magic[0],hdr.magic[1],hdr.magic[2]);
/*
					if(dataPoints > 0){
						uint8_t echoData[dataPoints];

						deviceFile.read((char*)echoData,dataPoints);

						ROS_INFO("Read data points");
					}
*/
					uint8_t terminationCharacter;
 
					do{
						ROS_INFO("ding");
						read(deviceFile,&terminationCharacter,sizeof(uint8_t));
					} while(terminationCharacter != 0xFC);
				}
				else{
					ROS_ERROR("Cannot read return data header (%d bytes read)",nbBytes);
				}
			}
			else{
				ROS_ERROR("Cannot write switch data command (%d bytes written)",nbBytes);
			}
			return 42.0;
		}

	private:

		ros::NodeHandle node;
		ros::Publisher sonarTopic;

		std::string   devicePath;
		int deviceFile = -1;

		uint32_t sequenceNumber;
};

int main(int argc,char ** argv){
	ros::init(argc,argv,"sonar_imagenex852");

	try{
		//TODO: parameterize path
		Imagenex852 sonar("/dev/sonar");

		sonar.run();
	}
	catch(std::exception &e){
		ROS_ERROR("Error: %s",e.what());
		return -1;
	}

	return 0;
}
