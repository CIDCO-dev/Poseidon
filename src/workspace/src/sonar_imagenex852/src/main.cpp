#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include <unistd.h>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <mutex>
#include <thread>

#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"

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
	uint8_t profileRange[2];
	uint8_t dataBytes[2];
} Imagenex852ReturnDataHeader;

#pragma pack()

class Imagenex852{
	public:
		Imagenex852(std::string devicePath) : devicePath(devicePath){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
			configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
			ROS_INFO("Fetching sonar configuration...");
			getConfiguration();
		}

		~Imagenex852(){
			if(deviceFile>=0) close(deviceFile);
		}

                void getConfiguration(){
                        char *    configKeys[] = {"sonarStartGain","sonarRange","sonarAbsorbtion","sonarPulseLength"};
                        uint8_t * valuePtrs[]  = {&sonarStartGain,&sonarRange,&sonarAbsorbtion,&sonarPulseLength};

                        for(int i=0;i<4;i++){
                                std::string valueString = getConfigValue(configKeys[i]);
                                setConfigValue(valueString, valuePtrs[i]);
                        }
                }

                std::string getConfigValue(std::string key){
                        setting_msg::ConfigurationService srv;

                        srv.request.key = key;

                        if(configurationClient.call(srv)){
                                return srv.response.value;
                        }
                        else{
                                return "";
                        }
                }

                void setConfigValue(const std::string & valStr,uint8_t * val){
                        mtx.lock();
                        sscanf(valStr.c_str(),"%hhu",val);
                        mtx.unlock();
                }

		void configurationChange(const setting_msg::Setting & setting){
			if(setting.key.compare("sonarStartGain")==0){
				setConfigValue(setting.value,&sonarStartGain);
			}
			else if(setting.key.compare("sonarRange")==0){
				setConfigValue(setting.value,&sonarRange);
			}
			else if(setting.key.compare("sonarAbsorbtion")==0){
				setConfigValue(setting.value,&sonarAbsorbtion);
			}
			else if(setting.key.compare("sonarPulseLength")==0){
				setConfigValue(setting.value,&sonarPulseLength);
			}
		}

		void processMessages(){
			ros::Subscriber sub = node.subscribe("configuration", 1000, &Imagenex852::configurationChange,this);
			ros::spin();
		}

		void run(){
			//Launch message pump
			std::thread t(&Imagenex852::processMessages,this);

			//open serial port
			deviceFile = open(devicePath.c_str(),O_RDWR);

			if(deviceFile < 0){
                               ROS_ERROR("Error while opening file %s : %s",devicePath.c_str(),strerror(errno));
                               throw std::invalid_argument(strerror(errno));
			}

			//Get serial port config
			struct termios tty;
			memset(&tty, 0, sizeof tty);

			if(tcgetattr(deviceFile, &tty) ==0) {

				//Set serial port config
				tty.c_cflag &= ~PARENB; //No parity
				tty.c_cflag &= ~CSTOPB; //One Stop bit
				tty.c_cflag |= CS8;     // 8 bits
				tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
				tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
				tty.c_lflag &= ~ICANON;
				tty.c_lflag &= ~ECHO; // Disable echo
				tty.c_lflag &= ~ECHOE; // Disable erasure
				tty.c_lflag &= ~ECHONL; // Disable new-line echo
				tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
				tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
				tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
				tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
				tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
				tty.c_cc[VTIME] = 250;    // Wait for up to 25s (250 deciseconds), returning as soon as any data is received.
				tty.c_cc[VMIN] = 0;

				cfsetispeed(&tty, B115200);
				cfsetospeed(&tty, B115200);

				if(tcsetattr(deviceFile,TCSANOW,&tty) == 0){
					ROS_INFO("Sonar file opened on %s",devicePath.c_str());

					ros::Rate error_rate( 1 );

				        while(ros::ok()){
	        	        		geometry_msgs::PointStamped msg;

			                	msg.header.seq=sequenceNumber++;
                				msg.header.stamp=ros::Time::now();
						msg.header.frame_id = "sonar";

						try{
	        		        		msg.point.z = measureDepth(500);
        	        				sonarTopic.publish(msg);
						}
						catch(std::exception & e){
							//ROS_ERROR already has been called. Lets sleep on this
							error_rate.sleep();
						}

						ros::spinOnce(); //XXX: This might not be necessary due to the processMessages() thread
        				}
				}
				else{
					ROS_ERROR("Error while configuring serial port %s : %s",devicePath.c_str(),strerror(errno));
					throw std::invalid_argument(strerror(errno));
				}
			}
			else{
 	                       ROS_ERROR("Error while fetching serial port configuration on %s : %s",devicePath.c_str(),strerror(errno));
                               throw std::invalid_argument(strerror(errno));
			}
		}

		int serialRead(uint8_t * buf,unsigned int sz){
			unsigned int totalRead = 0;

			while(totalRead < sz){
				unsigned int bytesRead = read(deviceFile,& (buf[totalRead]),sz - totalRead);

				if(bytesRead > 0){
	 				totalRead += bytesRead;
				}
				else if(bytesRead == 0){
					//File end
					return bytesRead;
				}
				else{
					return -1;
				}
			}

			return totalRead;
		}

		double measureDepth(int dataPoints){
			double depth = 0;

			Imagenex852SwitchDataCommand cmd;
			memset(&cmd,0,sizeof(Imagenex852SwitchDataCommand));

			mtx.lock();
                        cmd.range       = sonarRange;
                        cmd.startGain   = sonarStartGain;
                        cmd.absorption  = sonarAbsorbtion; //20 = 0.2db    675kHz
                        cmd.pulseLength = sonarPulseLength; //1-255 -> 1us to 255us in 1us increments
			mtx.unlock();

		        cmd.magic[0]    = 0xFE    ;
			cmd.magic[1]    = 0x44    ;
			cmd.headId      = 0x11    ;
			cmd.masterSlave = 0x43    ;

			cmd.profileMinimumRange =  0; //Min range in meters / 10
			cmd.triggerControl = 0x00;
			cmd.dataPoints  = dataPoints/10;
			cmd.profile     = 0;
			cmd.switchDelay = 0;
			cmd.frequency   = 0;
			cmd.terminationByte = 0xFD;

			unsigned int nbBytes;

			if( (nbBytes = write(deviceFile,&cmd,sizeof(Imagenex852SwitchDataCommand))) == 27){

				//ROS_INFO("Sent command (%d bytes)",nbBytes);

				Imagenex852ReturnDataHeader hdr;

				if( (nbBytes = serialRead((uint8_t*)&hdr,sizeof(Imagenex852ReturnDataHeader))) == 12){

					//ROS_INFO("Read %d bytes header",nbBytes);

					//ROS_INFO("%c%c%c",hdr.magic[0],hdr.magic[1],hdr.magic[2]);

					if(dataPoints > 0){
						uint8_t echoData[dataPoints];

						nbBytes = serialRead(echoData,dataPoints);

						if(nbBytes == dataPoints){
                                		        uint8_t terminationCharacter;

                                		        do{
                                                		serialRead(&terminationCharacter,sizeof(uint8_t));
                                        		} while(terminationCharacter != 0xFC);

                                        		uint16_t profileHigh = (hdr.profileRange[1] & 0x7E) >> 1;
                                        		uint16_t profileLow  = ((hdr.profileRange[1] & 0x01) << 7) | (hdr.profileRange[0] & 0x7F);

                                        		uint16_t depthCentimeters = (profileHigh << 8) | profileLow;

                                        		depth = (double)depthCentimeters / (double) 100;
						}
						else{
							ROS_ERROR("Could not read datapoints ( %d bytes read)",nbBytes);
							throw std::system_error();
						}
					}
				}
				else{
					ROS_ERROR("Cannot read return data header (%d bytes read)",nbBytes);
					throw std::system_error();
				}
			}
			else{
				ROS_ERROR("Cannot write switch data command (%d bytes written)",nbBytes);
				throw std::system_error();
			}
			return depth;
		}

	private:
		std::mutex mtx;
		uint8_t sonarStartGain = 0x06;
		uint8_t sonarRange = 32;
		uint8_t sonarAbsorbtion = 0x14; //20 = 0.2db    675kHz
		uint8_t sonarPulseLength= 150;

		ros::NodeHandle		node;
		ros::Publisher		sonarTopic;
		ros::ServiceClient	configurationClient;

		std::string   		devicePath;
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
