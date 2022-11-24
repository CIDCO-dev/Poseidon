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

class ImuSignalGenerator{
	public:
		ImuSignalGenerator(){
			imuTopic = node.advertise<sensor_msgs::Imu>("imu/data", 1000);
		}
		~ImuSignalGenerator(){}
		
		void publishMessage(uint32_t msgSequenceNumber,double yaw, double pitch, double roll){
			sensor_msgs::Imu msg;
			msg.header.seq=msgSequenceNumber;
			msg.header.stamp=ros::Time::now();

			/*
			terminate called after throwing an instance of 'tf2::LookupException'
  			what():  "base_link" passed to lookupTransform argument target_frame does not exist.
			*/
			imuTopic.publish(msg);
		}
	private:
		ros::NodeHandle node;
		ros::Publisher imuTopic;
};

class SonarSignalGenerator{
	public:
		SonarSignalGenerator(){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
		}
		~SonarSignalGenerator(){}
		
		void publishMessage(uint32_t msgSequenceNumber, double x, double y, double z){
			geometry_msgs::PointStamped msg;
			msg.header.seq=msgSequenceNumber;
			msg.header.stamp=ros::Time::now();
			msg.point.x = x;
			msg.point.y = y;
			msg.point.z = z;
			sonarTopic.publish(msg);
		}
	private:
		ros::NodeHandle node;
		ros::Publisher sonarTopic;
};

class LidarSignalGenerator{
	public:
		LidarSignalGenerator(){
			lidarTopic = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);
		}
		~LidarSignalGenerator(){}
		
		void publishMessage(uint32_t msgSequenceNumber, std::vector<geometry_msgs::Point32> points){
		
			sensor_msgs::PointCloud lidarXYZ;
			lidarXYZ.header.seq=msgSequenceNumber;
			lidarXYZ.header.stamp=ros::Time::now();
			
    		lidarXYZ.points = points;
			sensor_msgs::PointCloud2 msg;
			
			sensor_msgs::convertPointCloudToPointCloud2(lidarXYZ, msg);	
			
			lidarTopic.publish(msg);
			
		}
	private:
		ros::NodeHandle node;
		ros::Publisher lidarTopic;
};

class PoseidonBinaryReaderTest : public PoseidonBinaryReader{
	public:
		PoseidonBinaryReaderTest(std::string & filePath) : PoseidonBinaryReader(filePath){}
		~PoseidonBinaryReaderTest(){}
		
		void processGnss(PositionPacket & packet)override{
			gnssMessagesCount++;
			positions.push_back(packet);
		}
		
		void processImu(AttitudePacket & packet)override{
			imuMessagesCount++;
			atitudes.push_back(packet);
		}
		
		void processSonar(DepthPacket & packet)override{
			sonarMessagesCount++;
			depths.push_back(packet);
		}
		
		void processLidar(LidarPacket &packet)override{
			lidarMessagesCount++;
			laserPoints.push_back(packet);
		}
		
		std::vector<PositionPacket> getPositions(){
			return positions;
		}
		
		int getGnssMessagesCount(){
			return gnssMessagesCount;
		}
		
		std::vector<AttitudePacket> getAtitudes(){
			return atitudes;
		}
		
		int getImuMessagesCount(){
			return imuMessagesCount;
		}
		
		std::vector<DepthPacket> getDepths(){
			return depths;
		}
		
		int getSonarMessagesCount(){
			return sonarMessagesCount;
		}
		
		std::vector<LidarPacket> getLaserPoints(){
			return laserPoints;
		}
		
		int getLidarMessagesCount(){
			return lidarMessagesCount;
		}
		
		
	private:
		int gnssMessagesCount = 0;
		std::vector<PositionPacket> positions;
		
		int imuMessagesCount = 0;
		std::vector<AttitudePacket> atitudes;
		
		int sonarMessagesCount = 0;
		std::vector<DepthPacket> depths;
		
		int lidarMessagesCount = 0;
		std::vector<LidarPacket> laserPoints;
};
