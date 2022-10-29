#include <ros/ros.h>
#include <gtest/gtest.h>

#include <fstream> 
#include <filesystem>
//#include <unistd.h>

#include "loggerBinary.h"
#include "poseidon_reader.h"
#include "types.h"

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

class GnssSignalGenerator{
	public:
		GnssSignalGenerator(){
			gnssTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
		}
		~GnssSignalGenerator(){}
		
		void publishMessage(uint32_t msgSequenceNumber, double longitude, double latitude, double altitude){
			sensor_msgs::NavSatFix msg;
			msg.header.seq=msgSequenceNumber;
			msg.header.stamp=ros::Time::now();
			msg.status.service = 1;
			msg.status.status = 1;
			msg.header.stamp.nsec=0;
			msg.longitude=longitude;
			msg.latitude=latitude;
			msg.altitude=altitude;

			gnssTopic.publish(msg);
		}
		
		
	private:
		ros::NodeHandle node;
		ros::Publisher gnssTopic;

};

class PoseidonBinaryReaderTest : public PoseidonBinaryReader{
	public:
		PoseidonBinaryReaderTest(std::string & filePath) : PoseidonBinaryReader(filePath){}
		~PoseidonBinaryReaderTest(){}
		
		void processGnss(PacketHeader & hdr, PositionPacket & packet)override{
			gnssMessagesCount++;
			positions.push_back(packet);
		}
		
		std::vector<PositionPacket> getPositions(){
			return positions;
		}
		
		int getGnssMessagesCount(){
			return gnssMessagesCount;
		}
		
	private:
		int gnssMessagesCount = 0;
		std::vector<PositionPacket> positions;
		
};
